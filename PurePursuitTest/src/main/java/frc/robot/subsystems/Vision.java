package frc.robot.subsystems;

import java.util.ArrayList;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase
{
    VideoSource cam = new HttpCamera("Logitech Camera", "http://wpilibpi.local:1181/?action=stream");
    // VideoSource cam = CameraServer.getInstance().startAutomaticCapture("HP Camera", 0);
    CvSink cvSink = CameraServer.getInstance().getVideo(cam);
    CvSource outputStream = CameraServer.getInstance().putVideo("Processed Output", 640, 480);

    Mat source = new Mat();
    Mat output = new Mat();

    double x, y, areaPercentRatio;
    boolean valid;

    private void processImage()
    {
        Mat hsv = new Mat();
        Imgproc.cvtColor(source, hsv, Imgproc.COLOR_BGR2HSV);

        // preparing the mask to overlay 
        // https://www.geeksforgeeks.org/filter-color-with-opencv/
        Scalar lowerGreen = new Scalar(40, 75, 40);
        Scalar upperGreen = new Scalar(80, 255, 255);
        Mat mask = new Mat();
        Core.inRange(hsv, lowerGreen, upperGreen, mask);

        Mat opening = new Mat();
        Imgproc.morphologyEx(mask, opening, Imgproc.MORPH_OPEN, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));
        
        var contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(opening, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        
        output = opening; // for output stream

        if (contours.size() > 0)
        {
            var biggestContour = contours.get(0);
            for (int i = 0; i < contours.size(); i++)
            {
                if (contours.get(i).size().area() > biggestContour.size().area())
                    biggestContour = contours.get(i);
            }

            areaPercentRatio = 100 * Imgproc.contourArea(biggestContour) / source.size().area();

            // https://www.pyimagesearch.com/2016/02/01/opencv-center-of-contour
            var m = Imgproc.moments(biggestContour);
            x = m.m10 / m.m00;
            y = m.m01 / m.m00;
            
            // Convert corner-relative coordinates to center-relative
            x /= 640;
            y /= 480;

            x -= 0.5;
            y -= 0.5;

            y *= -1;

            // Convert to angles using FOV
            // https://www.chiefdelphi.com/t/lifecam-hd-3000-specifications-horizontal-and-vertical-fov/353550/2
            // https://github.com/rwb27/openflexure_microscope/wiki/Camera-Options
            x *= 46.1664822;
            y *= 27.0430565;

            valid = true;
        } else {
            x = 0;
            y = 0;
            areaPercentRatio = 0;
            valid = false;
        }
    }

    private void updateSmartDashboard()
    {
        SmartDashboard.putNumber("Vision X", getX());
        SmartDashboard.putNumber("Vision Y", getY());
        SmartDashboard.putNumber("Vision AreaPercentRatio", getA());
        SmartDashboard.putBoolean("Vision Valid", getV());
    }

    public double getX()
    {
        return x;
    }

    public double getY()
    {
        return y;
    }

    public double getA()
    {
        return areaPercentRatio;
    }

    public boolean getV()
    {
        return valid;
    }
    
    @Override
    public void periodic()
    {
        long errorCode = cvSink.grabFrame(source);
        
        if (errorCode == 0)
        {
            System.out.println(cvSink.getError());
            return;
        }

        processImage();
        updateSmartDashboard();
        
        outputStream.putFrame(output);
    }
}

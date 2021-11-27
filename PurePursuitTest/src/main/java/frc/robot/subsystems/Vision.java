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
    VideoSource logiCam = new HttpCamera("Logitech Camera", "http://wpilibpi.local:1181/?action=stream");
    CvSink cvSink = CameraServer.getInstance().getVideo(logiCam);
    CvSource outputStream = CameraServer.getInstance().putVideo("Processed Output", 640, 480);

    Mat source = new Mat();
    Mat output = new Mat();

    double x;
    double y;
    boolean valid;

    private void processImage()
    {
        // Filter out high frequency noise using a Gaussian Blur
        Mat blurred = new Mat();
        Imgproc.GaussianBlur(source, blurred, new Size(25, 25), 0);

        Mat hsv = new Mat();
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_BGR2HSV);

        // preparing the mask to overlay 
        // https://www.geeksforgeeks.org/filter-color-with-opencv/
        Scalar lowerGreen = new Scalar(40, 75, 40);
        Scalar upperGreen = new Scalar(80, 255, 255);
        Mat mask = new Mat();
        Core.inRange(hsv, lowerGreen, upperGreen, mask);
        
        var contours = new ArrayList<MatOfPoint>();
        var hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (contours.size() > 0)
        {
            // https://www.pyimagesearch.com/2016/02/01/opencv-center-of-contour
            var m = Imgproc.moments(contours.get(0));
            x = m.m10 / m.m00;
            y = m.m01 / m.m00;
            
            // Convert corner-relative coordinates to center-relative
            x /= 640;
            y /= 480;

            x -= 0.5;
            y -= 0.5;

            y *= -1;

            // Convert to 110 degree FOV scale
            x *= 110;
            y *= 110;

            valid = true;
        } else {
            x = 0;
            y = 0;
            valid = false;
        }

        output = mask;
    }

    private void updateSmartDashboard()
    {
        SmartDashboard.putNumber("Vision X", getX());
        SmartDashboard.putNumber("Vision Y", getY());
        SmartDashboard.putBoolean("Vision V", getV());
    }

    public double getX()
    {
        return x;
    }

    public double getY()
    {
        return y;
    }

    public boolean getV()
    {
        return valid;
    }
    
    @Override
    public void periodic()
    {
        //  update source frame
        long errorCode = cvSink.grabFrame(source);
        if (errorCode == 0)
            return;

        processImage();
        updateSmartDashboard();
        
        outputStream.putFrame(output);
    }
}

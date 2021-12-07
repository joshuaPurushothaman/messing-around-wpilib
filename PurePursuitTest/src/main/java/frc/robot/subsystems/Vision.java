package frc.robot.subsystems;

import java.util.ArrayList;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.geometry.*;
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
    Pose2d cameraPose = new Pose2d();
    Pose2d targetGlobalPose = new Pose2d();
    Transform2d cameraToRobotCenter = new Transform2d();

    // MedianFilter thetaFilter = new MedianFilter(5);
    LinearFilter thetaFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Vision");

    public Vision() {}

    public Vision(Transform2d cameraToRobotCenter, Pose2d targetGlobalPose)
    {
        setCameraTransform(cameraToRobotCenter);
        setTargetGlobalPose(targetGlobalPose);
    }

    public void setCameraTransform(Transform2d cameraToRobotCenter)
    {
        this.cameraToRobotCenter = cameraToRobotCenter;
    }

    public void setTargetGlobalPose(Pose2d targetGlobalPose)
    {
        this.targetGlobalPose = targetGlobalPose;
    }

    private void processImage()
    {
        Mat hsv = new Mat();
        Imgproc.cvtColor(source, hsv, Imgproc.COLOR_BGR2HSV);

        // https://www.geeksforgeeks.org/filter-color-with-opencv/
        Scalar lowerGreen = new Scalar(40, 70, 60);
        Scalar upperGreen = new Scalar(80, 255, 255);
        Mat mask = new Mat();
        Core.inRange(hsv, lowerGreen, upperGreen, mask);

        // Removes noise.
        Mat opening = new Mat();
        Imgproc.morphologyEx(mask, opening, Imgproc.MORPH_OPEN, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));
        
        var contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(opening, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        
        // Sets output
        Core.bitwise_and(source, source, output, opening);

        if (contours.size() > 0) {
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

            updateCameraPose(biggestContour, opening);

            valid = true;
        } else {
            x = 0;
            y = 0;
            areaPercentRatio = 0;
            cameraPose = new Pose2d();
            valid = false;
        }
    }

    private void updateCameraPose(MatOfPoint biggestContour, Mat frame)
    {
        double widthMm = 7.52;
        double heightMm = 15.27;
        double depthMm = 0;
        var objectPoints = new MatOfPoint3f(
            new Point3(-widthMm/2, heightMm/2, depthMm), // DL
            new Point3(widthMm/2, heightMm/2, depthMm), // DR
            new Point3(widthMm/2, -heightMm/2, depthMm), // UR
            new Point3(-widthMm/2, -heightMm/2, depthMm) // UL
        );
        
        var bRect = Imgproc.boundingRect(biggestContour);
        var imagePoints = new MatOfPoint2f(
            new Point(bRect.x, bRect.y + bRect.height),
            new Point(bRect.x + bRect.width, bRect.y + bRect.height),
            new Point(bRect.x + bRect.width, bRect.y),
            new Point(bRect.x, bRect.y)
        );

        // Camera internals
        double focalLength = frame.size(1);
        // Point center = new Point(frame.size(1) / 2, frame.size(0) / 2);
        Point center = new Point(frame.size(0) / 2, frame.size(1) / 2);
        Mat cameraMatrix = matFrom2DArray(new double[][]
            {{focalLength, 0, center.x},
            {0, focalLength, center.y},
            {0, 0, 1}}
        );

        // solvePnP will assume no distortion due to this
        MatOfDouble distCoeffs = new MatOfDouble();

        Mat rVec = new Mat();
        Mat tVec = new Mat();
        // SolvePnP IPPE method is optimized for coplanar image points
        Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rVec, tVec, false, Calib3d.SOLVEPNP_IPPE);

        // adjusting coordinate conventions...
        double camX = - tVec.get(2, 0)[0] / 1000; // Vision Z is WPILib Pose X
        double camY = tVec.get(0, 0)[0] / 1000; // Vision X is WPILib Pose Y
        
        var camRot = new Rotation2d(eulerAngles(rVec)[1]); // Vision Pose Y angle is WPILib Pose Theta


        cameraPose = new Pose2d(camX, camY, camRot);
        
    }

    // http://www.java2s.com/example/java-src/pkg/org/usfirst/frc/team2084/cmonster2016/vision/target-84cd3.html
    private static Mat matFrom2DArray(double[][] array) {
        Mat mat = new Mat(array.length, array[0].length, CvType.CV_64F);

        for (int r = 0; r < array.length; r++) {
            mat.put(r, 0, array[r]);
        }
        return mat;
    }

    private double[] eulerAngles(Mat rVec)
    {
        Mat rMat = new Mat();
        
        Calib3d.Rodrigues(rVec, rMat);
        
        var angles = Calib3d.RQDecomp3x3(rMat, new Mat(), new Mat());

        return angles;
    }

    private void updateTable()
    {
        table.getEntry("X").setNumber(getX());
        table.getEntry("Y").setNumber(getY());
        table.getEntry("Area").setNumber( getArea());
        table.getEntry("Valid").setBoolean( getValid());
        table.getEntry("PoseX").setNumber(getPose().getX());
        table.getEntry("PoseY").setNumber(getPose().getY());
        table.getEntry("PoseTheta").setNumber(getPose().getRotation().getDegrees());
    }

    public double getX()
    {
        return x;
    }

    public double getY()
    {
        return y;
    }

    public double getArea()
    {
        return areaPercentRatio;
    }

    public Pose2d getPose()
    {
        // I'm not sure if the ordering of all this is correct...
        return cameraPose.transformBy(cameraToRobotCenter).relativeTo(targetGlobalPose);
    }

    public boolean getValid()
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
        updateTable();
        
        outputStream.putFrame(output);
        output = new Mat(); // reset output mat
    }
}

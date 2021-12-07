package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision2 extends SubsystemBase
{
    double tx, ty, ta;
    boolean tv;
    Pose2d cameraPose = new Pose2d();
    Pose2d targetGlobalPose = new Pose2d();
    Transform2d cameraToRobotCenter = new Transform2d();

    NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Vision");
    NetworkTable visionPoseTable = visionTable.getSubTable("Vision");

    public Vision2(Transform2d cameraToRobotCenter, Pose2d targetGlobalPose)
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

    public double getX()
    {
        return tx;
    }

    public double getY()
    {
        return ty;
    }

    public double getArea()
    {
        return ta;
    }

    public boolean getValid()
    {
        return tv;
    }

    public Pose2d getPose()
    {
        // I'm not sure if the ordering of all this is correct...
        return cameraPose.transformBy(cameraToRobotCenter).relativeTo(targetGlobalPose);
    }

    @Override
    public void periodic()
    {
        updateTable();
    }

    private void updateTable()
    {
        tx = visionTable.getEntry("X").getDouble(0);
        ty = visionTable.getEntry("Y").getDouble(0);
        ta = visionTable.getEntry("Area").getDouble(0);
        tv = visionTable.getEntry("Valid").getBoolean(false);
        
        double pX = visionPoseTable.getEntry("X").getDouble(0);
        double pY = visionPoseTable.getEntry("Y").getDouble(0);
        double pThetaDegrees = visionPoseTable.getEntry("Theta Degrees").getDouble(0);

        cameraPose = new Pose2d(pX, pY, Rotation2d.fromDegrees(pThetaDegrees));
    }
}

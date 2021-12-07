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

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Vision");

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
        tx = table.getEntry("X").getDouble(0);
        ty = table.getEntry("Y").getDouble(0);
        ta = table.getEntry("Area").getDouble(0);
        tv = table.getEntry("Valid").getBoolean(false);
        
        double pX = table.getEntry("PoseX").getDouble(0);
        double pY = table.getEntry("PoseY").getDouble(0);
        double pThetaDegrees = table.getEntry("PoseTheta").getDouble(0);

        cameraPose = new Pose2d(pX, pY, Rotation2d.fromDegrees(pThetaDegrees));
    }
}

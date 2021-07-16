package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TrajDrivetrain extends Drivetrain
{
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
	DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.69); //  TODO: tune trackwidth
	
	
    
    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds()
    {
        return new DifferentialDriveWheelSpeeds(lfEncoder.getVelocity(), rfEncoder.getVelocity());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts)
    {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(-rightVolts);
        dt.feed();
    }

    public DifferentialDriveKinematics getKinematics()
    {
		return kinematics;
    }
    
    public void resetOdometry(Pose2d poseMeters)
    {
        odometry.resetPosition(poseMeters, gyro.getRotation2d());
    }
    
	@Override
	public void periodic()
	{
		// prints to SmartDashboard, periodically updating
		SmartDashboard.putNumber("lf position", lfEncoder.getPosition());
		SmartDashboard.putNumber("rf position", rfEncoder.getPosition());

		SmartDashboard.putNumber("lf Velocity", lfEncoder.getVelocity());
        SmartDashboard.putNumber("rf Velocity", rfEncoder.getVelocity());
        
        // Update the odometry in the periodic block
		odometry.update(gyro.getRotation2d(), lfEncoder.getPosition(), rfEncoder.getPosition());
	}
}

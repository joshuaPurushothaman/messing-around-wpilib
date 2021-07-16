package frc.robot.subsystems;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

import java.io.IOException;
import java.nio.file.Path;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import static com.revrobotics.ControlType.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.trajectory.*;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import frc.robot.Constants;

//ToDO: chassisspeeds??? check wpilib examples...
public class Mecanum
{
	CANSparkMax lf = new CANSparkMax(Constants.LF_MOTOR_ID, kBrushless);
	CANSparkMax lb = new CANSparkMax(Constants.LB_MOTOR_ID, kBrushless);
	CANSparkMax rf = new CANSparkMax(Constants.RF_MOTOR_ID, kBrushless);
	CANSparkMax rb = new CANSparkMax(Constants.RB_MOTOR_ID, kBrushless);

	CANEncoder lfEncoder = lf.getEncoder();
	CANEncoder lbEncoder = lb.getEncoder();
	CANEncoder rfEncoder = rf.getEncoder();
	CANEncoder rbEncoder = rb.getEncoder();

	CANPIDController lfPid = lf.getPIDController();
	CANPIDController lbPid = lb.getPIDController();
	CANPIDController rfPid = rf.getPIDController();
	CANPIDController rbPid = rb.getPIDController();

	MecanumDrive dt = new MecanumDrive(lf, lb, rf, rb);
	
	Gyro gyro = new ADXRS450_Gyro();
	Pose2d pose;

	// Creating my kinematics object using the module locations relative to the robot center.
	MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
		new Translation2d(0.25, 0.25),  // LF
		new Translation2d(0.25, -0.25), // RF
		new Translation2d(-0.25, 0.25), // LB
		new Translation2d(-0.25, -0.25) // RB
	);    

	MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, new Rotation2d(Math.toDegrees(gyro.getAngle())));
    
	HolonomicDriveController controller = new HolonomicDriveController(	
																		new PIDController(1, 0, 0),
																		new PIDController(1, 0, 0),
																		new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(0.5, 0.25))
																	  );
    
    Trajectory trajectory;

	Limelight limelight;
	PIDController aimXPID = new PIDController(1/27, 0, 0);   // ToDO:  TUNE ME and check units bc m/s
	PIDController aimRotPID = new PIDController(1/27, 0, 0);   // ToDO:  TUNE ME
	PIDController distPID = new PIDController(1/20, 0, 0);   // ToDO:  TUNE ME

	public Mecanum()
	{
		gyro.reset();

		lfEncoder.setVelocityConversionFactor(1);   //  ToDO: all encoders, pos AND vel

		/*
		* --- WARNING!!! ---
		* The following code means that you HAVE to set up the DS in the SAME spot the robot is placed, 
		* because the DS position determines the chosen trajectory.
		* Seems normal but just specifying... When testing, just choose it in the DS app.
		*/
		String path = Integer.toString(DriverStation.getInstance().getLocation());

		// OPENING TRAJECTORY
		try {
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("PathWeaver/output/" + path);
			trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		} catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
		}
	}

	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative)
	{
		if (fieldRelative) {
		  dt.driveCartesian(ySpeed, xSpeed, rot, -gyro.getAngle());
		} else {
		  dt.driveCartesian(ySpeed, xSpeed, rot);
		}
	
	}

	public void followTrajectory(double time)
	{
		var desiredState = trajectory.sample(time);
		
		ChassisSpeeds targetChassisSpeeds = controller.calculate(pose, desiredState,
			trajectory.getStates().get(trajectory.getStates().size() - 1)   //  these two lines here
			.poseMeters.getRotation()                      //  keep the swerve rotation the same lol
		);                                     
		
		//  pulled that from constructor of SwerveControllerCommand lol
		
		var targetWheelSpeeds = kinematics.toWheelSpeeds(targetChassisSpeeds);

		lfPid.setReference(targetWheelSpeeds.frontLeftMetersPerSecond, kVelocity);
		lbPid.setReference(targetWheelSpeeds.rearLeftMetersPerSecond, kVelocity);
		rfPid.setReference(targetWheelSpeeds.frontRightMetersPerSecond, kVelocity);
		rbPid.setReference(targetWheelSpeeds.rearRightMetersPerSecond, kVelocity);
	}

	public void aim()
	{
		drive(  //  make sure to consider units... this is not based on power            
			aimXPID.calculate(limelight.getTx(), 0),
			distPID.calculate(limelight.getTy(), 0),
			aimRotPID.calculate(limelight.getTx(), 0),
			false
		);
	}

	/**
	 * call me during periodic pls ;)
	 */
	public void updateOdometry()
	{
		// Get my gyro angle. We are negating the value because gyros return positive
		// values as the robot turns clockwise. This is not standard convention that is
		// used by the WPILib classes.
		var gyroAngle = Rotation2d.fromDegrees(-gyro.getAngle());

		// Update the pose
		pose = odometry.update(gyroAngle, 
			new MecanumDriveWheelSpeeds(
				lfEncoder.getVelocity(),
				lbEncoder.getVelocity(),
				rfEncoder.getVelocity(),
				rbEncoder.getVelocity()
			)
		);
	}
}
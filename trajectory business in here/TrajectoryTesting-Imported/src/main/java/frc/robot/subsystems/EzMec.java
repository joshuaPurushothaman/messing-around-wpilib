package frc.robot.subsystems;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;

import frc.robot.Constants;

//ToDO: chassisspeeds??? check wpilib examples...
public class EzMec
{
	CANSparkMax lf = new CANSparkMax(Constants.LF_MOTOR_ID, kBrushless);
	CANSparkMax lb = new CANSparkMax(Constants.LB_MOTOR_ID, kBrushless);
	CANSparkMax rf = new CANSparkMax(Constants.RF_MOTOR_ID, kBrushless);
	CANSparkMax rb = new CANSparkMax(Constants.RB_MOTOR_ID, kBrushless);

	CANEncoder lfEncoder = lf.getEncoder();
	CANEncoder lbEncoder = lb.getEncoder();
	CANEncoder rfEncoder = rf.getEncoder();
	CANEncoder rbEncoder = rb.getEncoder();

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
    
	Limelight limelight;
	PIDController aimXPID = new PIDController(1/27, 0, 0);   // ToDO:  TUNE ME and check units bc m/s
	PIDController aimRotPID = new PIDController(1/27, 0, 0);   // ToDO:  TUNE ME
	PIDController distPID = new PIDController(1/20, 0, 0);   // ToDO:  TUNE ME

	public EzMec()
	{
		gyro.reset();
		gyro.calibrate();

		// wheel diameter is 6 inches, so circumference is 6*pi in (ie, the robot moves 6pi in per 1 rev of the wheels)
		lfEncoder.setPositionConversionFactor(6 * Math.PI);
		rfEncoder.setPositionConversionFactor(6 * Math.PI);

		lf.burnFlash();
		lb.burnFlash();
		rf.burnFlash();
		rb.burnFlash();
	}

	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative)
	{
		if (fieldRelative) {
		  dt.driveCartesian(ySpeed, xSpeed, rot, -gyro.getAngle());
		} else {
		  dt.driveCartesian(ySpeed, xSpeed, rot);
		}
	
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
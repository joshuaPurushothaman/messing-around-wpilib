package frc.robot.subsystems;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import edu.wpi.first.wpilibj.controller.PIDController;

import edu.wpi.first.wpilibj.drive.MecanumDrive;

import frc.robot.Constants;

public class EzMecNODOM
{
	CANSparkMax lf = new CANSparkMax(Constants.LF_MOTOR_ID, kBrushless);
	CANSparkMax lb = new CANSparkMax(Constants.LB_MOTOR_ID, kBrushless);
	CANSparkMax rf = new CANSparkMax(Constants.RF_MOTOR_ID, kBrushless);
	CANSparkMax rb = new CANSparkMax(Constants.RB_MOTOR_ID, kBrushless);

	CANEncoder lfEncoder = lf.getEncoder();
	CANEncoder rfEncoder = rf.getEncoder();

	MecanumDrive dt = new MecanumDrive(lf, lb, rf, rb);
	
	Gyro gyro = new ADXRS450_Gyro();
    
	Limelight limelight;
	PIDController aimPID = new PIDController(0.01, 0, 0);

	//#region AUTON PIDS
	PIDController distPID = new PIDController(1/15, 0, 0);
	PIDController turnPID = new PIDController(0.01, 0, 0); // This means .1 power at 10 degrees, full power at >100 degrees...
	//#endregion

	public EzMecNODOM()
	{
		calibrate();

		// wheel diameter is 6 inches, so circumference is 6*pi in (ie, the robot moves 6pi in per 1 rev of the wheels)
		lfEncoder.setPositionConversionFactor(6 * Math.PI);
		rfEncoder.setPositionConversionFactor(6 * Math.PI);

		lf.burnFlash();
		lb.burnFlash();
		rf.burnFlash();
		rb.burnFlash();
	}

	/**
	 * @param distance to drive in feet, forward is positive
	 */
	public void driveToDistance(double distance)
	{
		double curDist = (lfEncoder.getPosition() + rfEncoder.getPosition()) / 2;
		
		drive
		(
			0,
			distPID.calculate(curDist, distance),
			turnPID.calculate(gyro.getAngle(), 0),
			false
		);
	}

	/**
	 * Turns to an angle relative to the robot's starting angle.
	 * <p>Example: if you're facing East, turnToAngle(90) wouldn't do anything.
	 * If your intention was to turn from East to South, you would call turnToAngle(180).
	 * @param angle to turn to in degrees, right is positive
	 */
	public void turnToAngle(double angle)
	{
		drive(0, 0, turnPID.calculate(gyro.getAngle(), angle), false);
	}

	public void aim()
	{
		drive(0, 0, aimPID.calculate(limelight.getTx(), 0), false);
	}
	

	/**
	 * Drive method for Mecanum platform.
	 * Angles are measured clockwise from the positive X axis. The robot's speed is independent from its angle or rotation rate.
	 * 
	 * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
	 * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Right is positive.
	 * @param rot The robot's rotation rate around the Z axis
	 * @param fieldRelative Whether to use field-relative controls or not.
	 * In this mode, going forwards will always go forwards relative to the starting orientation of the robot.
	 */
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative)
	{
		if (fieldRelative) {
		  dt.driveCartesian(ySpeed, xSpeed, rot, -gyro.getAngle());
		} else {
		  dt.driveCartesian(ySpeed, xSpeed, rot);
		}
	}

	public void calibrate()
	{
		gyro.reset();
		gyro.calibrate();
	}

	public void displayEncoderTicks()
	{
		SmartDashboard.putNumber("left enc", lfEncoder.getPosition());
		SmartDashboard.putNumber("right enc", rfEncoder.getPosition());
	}
}
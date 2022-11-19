// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
	protected static final double kCountsPerRevolution = 360;
	protected static final double kWheelDiameterMeters = Units.inchesToMeters(6);
	protected static final double kTrackWidthMeters = Units.inchesToMeters(26);
	public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
	protected static final double kMaxOutputFast = 1.0;
	protected static final double kMaxOutputSlow = 0.5;

	CANSparkMax leftMotor = new CANSparkMax(Constants.LEFT_MOTOR_ID, MotorType.kBrushless);
	CANSparkMax rightMotor = new CANSparkMax(Constants.RIGHT_MOTOR_ID, MotorType.kBrushless);

	RelativeEncoder leftEncoder = leftMotor.getEncoder();
	RelativeEncoder rightEncoder = rightMotor.getEncoder();

	DifferentialDrive dt;

	DifferentialDriveOdometry odom;
	
	Field2d field = new Field2d();

	ADXRS450_Gyro gyro = new ADXRS450_Gyro();

	public Drivetrain() {
		// Use METERS as unit for encoder distances
		leftEncoder.setPositionConversionFactor((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
		rightEncoder.setPositionConversionFactor((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
		leftEncoder.setVelocityConversionFactor((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
		rightEncoder.setVelocityConversionFactor((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
		
		leftMotor.setInverted(false);
		rightMotor.setInverted(true);

		resetEncoders();

		leftMotor.burnFlash();
		rightMotor.burnFlash();

		dt = new DifferentialDrive(leftMotor, rightMotor);
		odom = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-getHeading()), new Pose2d());

		SmartDashboard.putData(field);
	}
	
	public void arcadeDrive(double xSpeed, double zRotation) {
		arcadeDrive(xSpeed, zRotation, true);
	}

	public void arcadeDrive(double xSpeed, double zRotation, boolean slowMode) {
		if (slowMode)
			setMaxOutput(kMaxOutputSlow);
		else
			setMaxOutput(kMaxOutputFast);
		
		dt.arcadeDrive(xSpeed, zRotation);
	}

	PIDController leftController = new PIDController(Constants.kPSetWheelSpeeds, 0, 0);
	SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter);
	PIDController rightController = new PIDController(Constants.kPSetWheelSpeeds, 0, 0);

	/**
	 * Meters per second
	 * @param left
	 * @param right
	 */
	public void setWheelSpeeds(double left, double right)
	{
		var curSpeeds = getWheelSpeeds();

		double leftSpeed = leftController.calculate(curSpeeds.leftMetersPerSecond, left) + feedforward.calculate(left);
		double rightSpeed = rightController.calculate(curSpeeds.rightMetersPerSecond, right) + feedforward.calculate(right);

		tankDriveVolts(leftSpeed, rightSpeed);
	}

	/**
   	 * Controls the left and right sides of the drive directly with voltages.
   	 * @param leftVolts the commanded left output
   	 * @param rightVolts the commanded right output
   	 */
	public void tankDriveVolts(double leftVolts, double rightVolts) {
		leftMotor.setVoltage(leftVolts);
		rightMotor.setVoltage(-rightVolts); // We invert this to maintain +ve = forward
		dt.feed();
	}

	public void resetEncoders() {
		leftEncoder.setPosition(0);
		rightEncoder.setPosition(0);
	}

	public double getDistanceMeters() {
		return Math.max(leftEncoder.getPosition(), rightEncoder.getPosition());
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds()
	{
		return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
	}

	public double getHeading()
	{
		return gyro.getAngle();
	}

	public void resetGyro()
	{
		gyro.reset();
	}

	public Pose2d getPose()
	{
		return odom.getPoseMeters();
	}

	public void resetPose(Pose2d pose)
	{
		odom.resetPosition(pose, Rotation2d.fromDegrees(-getHeading()));
		resetEncoders();
	}

	public void resetSensors()
	{
		resetGyro();
		resetEncoders();
	}

	public void setMaxOutput(double maxOutput)
	{
		dt.setMaxOutput(maxOutput);
	}

	@Override
	public void periodic()
	{
		odom.update(
			Rotation2d.fromDegrees(-getHeading()),
			leftEncoder.getPosition(),
			rightEncoder.getPosition()
		);
		
		field.setRobotPose(getPose());
		
		// update PID just in case I do a Hot Reload in VSCode while debugging
		// leftController.setP(Constants.kP);
		// rightController.setP(Constants.kP);

		SmartDashboard.putNumber("x", getPose().getX());
		SmartDashboard.putNumber("y", getPose().getY());
		SmartDashboard.putNumber("theta", getPose().getRotation().getDegrees());
		
		var curSpeeds = getWheelSpeeds();
		SmartDashboard.putNumber("Left Speed", curSpeeds.leftMetersPerSecond);
		SmartDashboard.putNumber("Right Speed", curSpeeds.rightMetersPerSecond);
	}
}

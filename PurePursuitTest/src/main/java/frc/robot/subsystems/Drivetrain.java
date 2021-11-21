// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
	private static final double kCountsPerRevolution = 1437.0;
	private static final double kWheelDiameterMeters = 0.070;
	private static final double kTrackWidthMeters = 0.141;
	public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

	// The Romi has the left and right motors set to
	// PWM channels 0 and 1 respectively
	private final Spark left = new Spark(0);
	private final Spark right = new Spark(1);

	// The Romi has onboard encoders that are hardcoded
	// to use DIO pins 4/5 and 6/7 for the left and right
	private final Encoder leftEncoder = new Encoder(4, 5);
	private final Encoder rightEncoder = new Encoder(6, 7);

	// Set up the differential drive controller
	private final DifferentialDrive drive = new DifferentialDrive(left, right);

	// Set up the RomiGyro
	private final RomiGyro gyro = new RomiGyro();

	// Set up the BuiltInAccelerometer
	private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();


	private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d());

	// private final Field2d field = new Field2d();


	/** Creates a new Drivetrain. */
	public Drivetrain() {
		// Use METERS as unit for encoder distances
		leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
		rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);

		resetEncoders();

		// SmartDashboard.putData(field);
	}

	public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
		drive.arcadeDrive(xaxisSpeed, zaxisRotate);
	}

	PIDController leftController = new PIDController(Constants.kPSetWheelSpeeds, 0, 0);
	SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(Constants.lksVolts, Constants.lkvVoltSecondsPerMeter);
	PIDController rightController = new PIDController(Constants.kPSetWheelSpeeds, 0, 0);
	SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(Constants.rksVolts, Constants.rkvVoltSecondsPerMeter);

	/**
	 * Meters per second
	 * @param left
	 * @param right
	 */
	public void setWheelSpeeds(double left, double right)
	{
		var curSpeeds = getWheelSpeeds();

		System.out.println(curSpeeds);

		double leftSpeed = leftController.calculate(curSpeeds.leftMetersPerSecond, left) + leftFF.calculate(left);
		double rightSpeed = rightController.calculate(curSpeeds.rightMetersPerSecond, right) + rightFF.calculate(right);

		tankDriveVolts(leftSpeed, rightSpeed);
	}

	/**
   	 * Controls the left and right sides of the drive directly with voltages.
   	 * @param leftVolts the commanded left output
   	 * @param rightVolts the commanded right output
   	 */
	public void tankDriveVolts(double leftVolts, double rightVolts) {
		left.setVoltage(leftVolts);
		right.setVoltage(-rightVolts); // We invert this to maintain +ve = forward
		drive.feed();
	}

	public void resetEncoders() {
		leftEncoder.reset();
		rightEncoder.reset();
	}

	public double getAverageDistanceMeters() {
		return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds()
	{
		return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
	}

	public double getHeading()
	{
		return gyro.getAngleZ();
	}

	/** Reset the gyro. */
	public void resetGyro() {
		gyro.reset();
	}

	public Pose2d getPose()
	{
		return odometry.getPoseMeters();
	}
	
	public void resetPose()
	{
		odometry.resetPosition(new Pose2d(), new Rotation2d());
		resetEncoders();
	}

	public void resetPose(Pose2d pose)
	{
		odometry.resetPosition(pose, Rotation2d.fromDegrees(gyro.getAngleZ()));
		resetEncoders();
	}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		odometry.update(new Rotation2d(getHeading()), leftEncoder.getDistance(),
						rightEncoder.getDistance());
		
		// field.setRobotPose(odometry.getPoseMeters());
		
		// update PID just in case I do a Hot Reload in VSCode while debugging
		// leftController.setP(Constants.kP);
		// rightController.setP(Constants.kP);

		SmartDashboard.putNumber("x", odometry.getPoseMeters().getX());
		SmartDashboard.putNumber("y", odometry.getPoseMeters().getY());
		SmartDashboard.putNumber("theta", odometry.getPoseMeters().getRotation().getDegrees());
	}

	public void resetSensors()
	{
		resetGyro();
		resetEncoders();
	}
}

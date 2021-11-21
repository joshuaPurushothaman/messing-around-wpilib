// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.*;
import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
	private static final double kCountsPerRevolution = 1440.0;
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
	private final DifferentialDrive dt = new DifferentialDrive(left, right);

	// Set up the RomiGyro
	private final RomiGyro gyro = new RomiGyro();

	// Set up the BuiltInAccelerometer
	private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();


	private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d());


	/** Creates a new Drivetrain. */
	public Drivetrain() {
		resetSensors();

		// Use METERS as unit for encoder distances
		leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
		rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
	}

	public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
		dt.arcadeDrive(xaxisSpeed, zaxisRotate);
	}

	PIDController leftController = new PIDController(Constants.kPDriveVel, 0, 0);
	PIDController rightController = new PIDController(Constants.kPDriveVel, 0, 0);
	/**
	 * Meters per second
	 * @param left
	 * @param right
	 */
	public void setWheelSpeeds(double left, double right)
	{
		var curSpeeds = getWheelSpeeds();

		double leftSpeed = leftController.calculate(curSpeeds.leftMetersPerSecond, left);
		double rightSpeed = leftController.calculate(curSpeeds.rightMetersPerSecond, right);
		dt.tankDrive(leftSpeed, rightSpeed);
	}

	public void resetEncoders() {
		leftEncoder.reset();
		rightEncoder.reset();
	}

	public double getLeftDistanceMeters() {
		return leftEncoder.getDistance();
	}

	public double getRightDistanceMeters() {
		return rightEncoder.getDistance();
	}

	public double getAverageDistanceMeters() {
		return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds()
	{
		return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
	}
	/**
	 * The acceleration in the X-axis.
	 *
	 * @return The acceleration of the Romi along the X-axis in Gs
	 */
	public double getAccelX() {
		return accelerometer.getX();
	}

	/**
	 * The acceleration in the Y-axis.
	 *
	 * @return The acceleration of the Romi along the Y-axis in Gs
	 */
	public double getAccelY() {
		return accelerometer.getY();
	}

	/**
	 * The acceleration in the Z-axis.
	 *
	 * @return The acceleration of the Romi along the Z-axis in Gs
	 */
	public double getAccelZ() {
		return accelerometer.getZ();
	}

	/**
	 * Current angle of the Romi around the X-axis.
	 *
	 * @return The current angle of the Romi in degrees
	 */
	public double getGyroAngleX() {
		return gyro.getAngleX();
	}

	/**
	 * Current angle of the Romi around the Y-axis.
	 *
	 * @return The current angle of the Romi in degrees
	 */
	public double getGyroAngleY() {
		return gyro.getAngleY();
	}

	/**
	 * Current angle of the Romi around the Z-axis.
	 *
	 * @return The current angle of the Romi in degrees
	 */
	public double getGyroAngleZ() {
		return gyro.getAngleZ();
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

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		odometry.update(new Rotation2d(getHeading()), leftEncoder.getDistance(),
						rightEncoder.getDistance());
	}

	public void resetSensors() 
	{
		resetEncoders();
		resetGyro();
	}

	public void tankDriveVolts(double leftVolts, double rightVolts)
	{
		left.setVoltage(leftVolts);
		right.setVoltage(rightVolts);
		
		dt.feed();
	}
}

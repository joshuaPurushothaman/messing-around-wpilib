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
	private final Spark m_leftMotor = new Spark(0);
	private final Spark m_rightMotor = new Spark(1);

	// The Romi has onboard encoders that are hardcoded
	// to use DIO pins 4/5 and 6/7 for the left and right
	private final Encoder m_leftEncoder = new Encoder(4, 5);
	private final Encoder m_rightEncoder = new Encoder(6, 7);

	// Set up the differential drive controller
	private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

	// Set up the RomiGyro
	private final RomiGyro m_gyro = new RomiGyro();

	// Set up the BuiltInAccelerometer
	private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();


	private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d());


	/** Creates a new Drivetrain. */
	public Drivetrain() {
		// Use METERS as unit for encoder distances
		m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
		m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
		resetEncoders();
	}

	public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
		m_drive.arcadeDrive(xaxisSpeed, zaxisRotate);
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
		m_drive.tankDrive(leftSpeed, rightSpeed);
	}

	public void resetEncoders() {
		m_leftEncoder.reset();
		m_rightEncoder.reset();
	}

	public double getLeftDistanceMeters() {
		return m_leftEncoder.getDistance();
	}

	public double getRightDistanceMeters() {
		return m_rightEncoder.getDistance();
	}

	public double getAverageDistanceMeters() {
		return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds()
	{
		return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
	}
	/**
	 * The acceleration in the X-axis.
	 *
	 * @return The acceleration of the Romi along the X-axis in Gs
	 */
	public double getAccelX() {
		return m_accelerometer.getX();
	}

	/**
	 * The acceleration in the Y-axis.
	 *
	 * @return The acceleration of the Romi along the Y-axis in Gs
	 */
	public double getAccelY() {
		return m_accelerometer.getY();
	}

	/**
	 * The acceleration in the Z-axis.
	 *
	 * @return The acceleration of the Romi along the Z-axis in Gs
	 */
	public double getAccelZ() {
		return m_accelerometer.getZ();
	}

	/**
	 * Current angle of the Romi around the X-axis.
	 *
	 * @return The current angle of the Romi in degrees
	 */
	public double getGyroAngleX() {
		return m_gyro.getAngleX();
	}

	/**
	 * Current angle of the Romi around the Y-axis.
	 *
	 * @return The current angle of the Romi in degrees
	 */
	public double getGyroAngleY() {
		return m_gyro.getAngleY();
	}

	/**
	 * Current angle of the Romi around the Z-axis.
	 *
	 * @return The current angle of the Romi in degrees
	 */
	public double getGyroAngleZ() {
		return m_gyro.getAngleZ();
	}

	public double getHeading()
	{
		return m_gyro.getAngleZ();
	}

	/** Reset the gyro. */
	public void resetGyro() {
		m_gyro.reset();
	}

	public Pose2d getPose()
	{
		return m_odometry.getPoseMeters();
	}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		m_odometry.update(new Rotation2d(getHeading()), m_leftEncoder.getDistance(),
						m_rightEncoder.getDistance());
	}
}

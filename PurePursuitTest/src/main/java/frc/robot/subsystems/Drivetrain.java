// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.*;

public class Drivetrain extends SubsystemBase {
	private static final double kCountsPerRevolution = 1437.0;
	private static final double kWheelDiameterMeters = 0.070;
	private static final double kTrackWidthMeters = 0.141;
	public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

	// The Romi has the left and right motors set to
	// PWM channels 0 and 1 respectively
	Spark left = new Spark(0);
	Spark right = new Spark(1);

	// The Romi has onboard encoders that are hardcoded
	// to use DIO pins 4/5 and 6/7 for the left and right
	Encoder leftEncoder = new Encoder(4, 5);
	Encoder rightEncoder = new Encoder(6, 7);

	// Set up the differential drive controller
	DifferentialDrive dt = new DifferentialDrive(left, right);

	// Set up the RomiGyro
	RomiGyro gyro = new RomiGyro();

	// Set up the BuiltInAccelerometer
	// BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

	DifferentialDrivePoseEstimator odometryEstimator;
	
	Field2d field = new Field2d();
	Pose2d startingPose;

	Vision2 vision;

	public Drivetrain(Vision2 vision, Pose2d startingPose) {
		// Use METERS as unit for encoder distances
		leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
		rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);

		resetEncoders();

		this.vision = vision;

		// odometryEstimator = new DifferentialDrivePoseEstimator(
		// 	new Rotation2d(-Units.degreesToRadians(getHeading())),
		// 	startingPose,
		// 	new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02), // State measurement standard deviations. X, Y, theta.
		// 	new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
		// 	new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)); // Global measurement standard deviations. X, Y, and theta.

			
		odometryEstimator = new DifferentialDrivePoseEstimator(
			new Rotation2d(-Units.degreesToRadians(getHeading())),
			startingPose,
			new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02), // State measurement standard deviations. X, Y, theta.
			new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
			new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)); // Global measurement standard deviations. X, Y, and theta.

		this.startingPose = startingPose;

		SmartDashboard.putData(field);
	}

	public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
		dt.arcadeDrive(xaxisSpeed, zaxisRotate);
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
		dt.feed();
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
		return odometryEstimator.getEstimatedPosition();
	}
	
	public void resetPose()
	{
		resetPose(startingPose);
	}

	public void resetPose(Pose2d pose)
	{
		odometryEstimator.resetPosition(pose, Rotation2d.fromDegrees(-getHeading()));
		resetEncoders();
	}

	@Override
	public void periodic() {
		// Uncommenting the below lines will enable vision based pose estimation
		// if (vision.getValid())
		// 	odometryEstimator.addVisionMeasurement(vision.getPose(), Timer.getFPGATimestamp());

		odometryEstimator.update(
			Rotation2d.fromDegrees(-getHeading()),
			getWheelSpeeds(), 
			leftEncoder.getDistance(),
			rightEncoder.getDistance()
		);
		
		field.setRobotPose(getPose());
		
		// update PID just in case I do a Hot Reload in VSCode while debugging
		// leftController.setP(Constants.kP);
		// rightController.setP(Constants.kP);

		SmartDashboard.putNumber("x", getPose().getX());
		SmartDashboard.putNumber("y", getPose().getY());
		SmartDashboard.putNumber("theta", getPose().getRotation().getDegrees());
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

	public Pose2d getStartingPose()
	{
		return startingPose;
	}
}

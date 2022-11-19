// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
	public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(13.88);
	public static final double kMaxAccelerationMetersPerSecondSquared = kMaxSpeedMetersPerSecond / 2;
	public static final double kMaxTurnSpeedDegreesPerSecond = 360.0 / 0.1;
	public static final double kMaxTurnAccelerationDegreesPerSecondSquared = kMaxTurnSpeedDegreesPerSecond / 0.01;

	public static final double kPDTD = 2.5;
	public static final double kToleranceMeters = 0.05;
	public static final double kPTTA = 0.05;
	public static final double kToleranceDegrees = 1;

	public static final double kPSetWheelSpeeds = 5;

	public static final double ksVolts = 0.3;
	public static final double kvVoltSecondsPerMeter = 1.8;

	public static final double kPLift = 10.0;
	public static final double kMaxSpeedMetersPerSecondLift = 1.75;
	public static final double kMaxAccelerationMetersPerSecondSquaredLift = kMaxSpeedMetersPerSecondLift / 2;
	public static final double kToleranceMetersLift = 0.05;


	public static final int LEFT_MOTOR_ID = 1;
	public static final int RIGHT_MOTOR_ID = 2;

	public static final int LIFT_MOTOR_ID = 3;

	public static final int LEFT_SHOOTER_MOTOR_ID = 4;
	public static final int RIGHT_SHOOTER_MOTOR_ID = 5;
}

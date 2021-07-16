// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
	public static final int LF_MOTOR_ID = 1;	//	TODO: enter motor CAN IDs
	public static final int RF_MOTOR_ID = 2;
	public static final int LB_MOTOR_ID = 3;
	public static final int RB_MOTOR_ID = 4;

	//	ADVANCED! These are example values. For trajectory folllowing. TODO: frc-robot-characterization
	public static final double ksVolts = 0.22;
	public static final double kvVoltSecondsPerMeter = 1.98;
	public static final double kaVoltSecondsSquaredPerMeter = 0.2;
	public static final double kMaxSpeedMetersPerSecond = 3;
	public static final double kMaxAccelerationMetersPerSecondSquared = 3;
	// Example value only - as above, this must be tuned for your drive!
	public static final double kPDriveVel = 8.5;
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import static edu.wpi.first.wpilibj.GenericHID.Hand.*;
import java.util.function.*;

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
    public static final class Drivetrain
    {
        public static final int lf = 0, rf = 1, lb = 2, rb = 3;
        
        public static final double SLOW_MODE_SPEED = 0.25,
                                    NON_SLOW_MODE_SPEED = 1;

		public static final double TRACK_WIDTH = 0.5;

		public static final double WHEEL_DIAMETER_METERS = 0.1524;
    }
    
    public static final class Controls
    {
        public static final XboxController driver = new XboxController(0),
                                            operator = new XboxController(1);

        public static final DoubleSupplier xSpeedAxis = () -> driver.getY(kLeft);
        public static final DoubleSupplier zRotationAxis = () -> driver.getX(kRight);
        public static final BooleanSupplier isQuickTurnButton = () -> driver.getBumper(kLeft);
        public static final BooleanSupplier isSlowModeButton = () -> driver.getTriggerAxis(kLeft) > 0.5;
    }
}

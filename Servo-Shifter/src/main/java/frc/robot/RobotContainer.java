// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
	Drivetrain dt = new Drivetrain();
	XboxController driverController = new XboxController(0);
	
	public RobotContainer()
	{
		var lowGearButton = new JoystickButton(driverController, XboxController.Button.kBumperLeft.value);
		var highGearButton = new JoystickButton(driverController, XboxController.Button.kBumperRight.value);

		lowGearButton
			.whileHeld(new RunCommand(() -> { dt.setAutomaticShifting(false); dt.shift(false); }, dt))
			.whenReleased(new RunCommand(() -> { dt.setAutomaticShifting(true); }, dt));

		highGearButton
			.whileHeld(new RunCommand(() -> { dt.setAutomaticShifting(false); dt.shift(true); }, dt))
			.whenReleased(new RunCommand(() -> { dt.setAutomaticShifting(true); }, dt));
		
		dt.setDefaultCommand(new RunCommand(() -> dt.arcadeDrive(driverController.getY(Hand.kLeft), driverController.getY(Hand.kRight)), dt));
	}
	
	public Command getAutonomousCommand()
	{
		return new DriveToDistance(1, dt);
	}
}

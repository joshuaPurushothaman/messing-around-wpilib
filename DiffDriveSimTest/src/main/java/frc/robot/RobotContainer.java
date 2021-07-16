// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.commands.AdvancedTrajectoryCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
	// The robot's subsystems and commands are defined here...
	DrivetrainSim dt = new DrivetrainSim();
	XboxController driverController = new XboxController(0);
	
	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer()
	{
		// Configure the button bindings
		configureButtonBindings();

		dt.setDefaultCommand(
			new RunCommand(
				() -> dt.drive(-driverController.getY(Hand.kLeft),
								driverController.getX(Hand.kRight)), dt));
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand()
	{
		// return new AdvancedTrajectoryCommand(dt);
		return new RunCommand(() -> dt.drive(1, 0)).withTimeout(15);
	}
}

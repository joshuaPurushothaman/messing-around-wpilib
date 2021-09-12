// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.commands.PurePursuit.FollowTrajCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final Drivetrain dt = new Drivetrain();
	private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);

	// Assumes a gamepad plugged into channnel 0
	private final Joystick m_controller = new Joystick(0);

	SendableChooser<Command> m_chooser = new SendableChooser<>();

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings()
	{
		dt.setDefaultCommand(new RunCommand(() -> dt.setWheelSpeeds(-m_controller.getRawAxis(1) * Constants.kMaxSpeedMetersPerSecond + m_controller.getRawAxis(0) * Constants.kMaxSpeedMetersPerSecond,
			-m_controller.getRawAxis(1) * Constants.kMaxSpeedMetersPerSecond - m_controller.getRawAxis(0) * Constants.kMaxSpeedMetersPerSecond), dt));
		
		// Example of how to use the onboard IO
		Button onboardButtonA = new Button(m_onboardIO::getButtonAPressed);
		onboardButtonA
				.whenActive(new PrintCommand("Button A Pressed"))
				.whenInactive(new PrintCommand("Button A Released"));

		m_chooser.setDefaultOption("Pure Pursuit", new FollowTrajCommand(dt));
		m_chooser.addOption("Set Wheel Speeds test", new RunCommand(() -> dt.setWheelSpeeds(0.3, 0.3), dt));
		m_chooser.addOption("Ramsete", new RamseteTrajCommand(dt));
		m_chooser.addOption("Sharp square",
					new DriveToDistanceCommand(0.3, dt).andThen(new TurnToAngleCommand(90, dt))
			.andThen(new DriveToDistanceCommand(0.3, dt).andThen(new TurnToAngleCommand(90, dt)))
			.andThen(new DriveToDistanceCommand(0.3, dt).andThen(new TurnToAngleCommand(90, dt)))
			.andThen(new DriveToDistanceCommand(0.3, dt).andThen(new TurnToAngleCommand(90, dt))));

		SmartDashboard.putData("Auto chooser", m_chooser);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand()
	{
		return m_chooser.getSelected();
	}

	public void disabledInit()
	{
		dt.resetPose();
		dt.resetGyro();
	}
}

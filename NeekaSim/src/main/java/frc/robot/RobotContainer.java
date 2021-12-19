// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.commands.*;
import frc.robot.commands.drive.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
	Drivetrain dt = new DrivetrainWithSim(new Pose2d(3, 3, Rotation2d.fromDegrees(0)));
	Lift lift = new Lift();

	// The robot's subsystems and commands are defined here...
	XboxController controller = new XboxController(0);

	SendableChooser<Command> chooser = new SendableChooser<>();

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer()
	{
		//#region Shorthand Commands
		var lowerLift = new SetLiftHeight(Lift.LOW_POSITION_METERS, lift);
		var upLift = new SetLiftHeight(Lift.HIGH_POSITION_METERS, lift);

		var teleopDriveCommand = new RunCommand(
			() -> dt.arcadeDrive(
				-controller.getRawAxis(1),
				controller.getRawAxis(0),
				controller.getRawButton(4)), dt);
		//#endregion Shorthand Commands

		//#region Button Bindings
		var elevatorUpButton = new POVButton(controller, 0);
		var oneEightyButton = new JoystickButton(controller, 3);

		elevatorUpButton
			.whileHeld(upLift);

		oneEightyButton
			.whenPressed(new OneEightyCommand(dt));
		//#endregion Button Bindings

		//#region Default Commands
		dt.setDefaultCommand(teleopDriveCommand);
		lift.setDefaultCommand(lowerLift.perpetually());
		//#endregion Default Commands

		//#region Auto Chooser Options
		chooser.setDefaultOption("Set Wheel Speeds test", new RunCommand(() -> dt.setWheelSpeeds(0.3, 0.3), dt));
		chooser.addOption("Ramsete", new RamseteTrajCommand(dt));
		chooser.addOption("Sharp square", new Square(dt));
		chooser.addOption("Profiled Square", new ProfiledSquare(dt));
		SmartDashboard.putData("Auto Chooser", chooser);
		//#endregion Auto Chooser Options
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand()
	{
		return chooser.getSelected();
	}

	public void disabledInit()
	{
		lift.set(0);
		dt.arcadeDrive(0, 0);
		dt.resetPose();
		dt.resetSensors();
	}
}

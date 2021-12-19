package frc.robot.commands.drive;

import java.util.List;

import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class RamseteTrajCommand extends SequentialCommandGroup
{
	public RamseteTrajCommand(Drivetrain dt)
	{       
		var autoVoltageConstraint =
			new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(Constants.ksVolts, 
											Constants.kvVoltSecondsPerMeter),
				Drivetrain.kinematics,
				10);

		// Create config for trajectory
		TrajectoryConfig config =
			new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
								Constants.kMaxAccelerationMetersPerSecondSquared)
				// Add kinematics to ensure max speed is actually obeyed
				.setKinematics(Drivetrain.kinematics)
				.addConstraint(autoVoltageConstraint);

		var start = dt.getStartingPose();

		Trajectory traj = TrajectoryGenerator.generateTrajectory(
			List.of(
				start,
				new Pose2d(1, 0, new Rotation2d(0)).relativeTo(start)
			),
			config
		);

		RamseteCommand ramseteCommand = new RamseteCommand(
			traj,
			dt::getPose,
			new RamseteController(),
			Drivetrain.kinematics,
			(left, right) -> dt.setWheelSpeeds(left, right),
			dt
		);
		
		addCommands
		(
			new InstantCommand(() -> dt.resetPose(traj.getInitialPose()), dt),
			new InstantCommand(() -> dt.resetSensors(), dt),
			ramseteCommand,
			new InstantCommand(() -> dt.setWheelSpeeds(0, 0), dt),
			new PrintCommand("\n\n\n\n\nTRAJ DONE\n\n\n\n\n")
		);
	}
}

package frc.robot.commands.PurePursuit;

import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajCommand extends SequentialCommandGroup
{
	public FollowTrajCommand(Drivetrain dt)
	{
		var autoVoltageConstraint =
			new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(Constants.rksVolts, 
											Constants.rkvVoltSecondsPerMeter),
				Drivetrain.kinematics,
				10);

			
		TrajectoryConfig config =
		new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
							Constants.kMaxAccelerationMetersPerSecondSquared)
			.setKinematics(Drivetrain.kinematics)
			.addConstraint(autoVoltageConstraint)
			.setStartVelocity(0.2)
			.setEndVelocity(0.0);

		var start = dt.getStartingPose();

		Trajectory traj = TrajectoryGenerator.generateTrajectory(
			List.of(
				start,
				new Pose2d(0.2, 0, new Rotation2d(0)).relativeTo(start)
			),
			config
		);

		PurePursuitCommand purePursuitCommand = new PurePursuitCommand
		(
			traj, dt::getPose, Drivetrain.kinematics.trackWidthMeters, dt::setWheelSpeeds, Constants.kLookAheadDistance, dt
		);

		addCommands
		(
			new InstantCommand(() -> purePursuitCommand.getController().reset()),
			new InstantCommand(() -> dt.resetPose(traj.getInitialPose()), dt),
			new InstantCommand(() -> dt.resetSensors(), dt),
			purePursuitCommand,
			new InstantCommand(() -> dt.setWheelSpeeds(0, 0), dt),
			new PrintCommand("\n\n\n\n\nTRAJ DONE\n\n\n\n\n")
		);
	}
}

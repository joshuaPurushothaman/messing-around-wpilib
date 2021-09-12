package frc.robot.commands.PurePursuit;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajCommand extends SequentialCommandGroup
{
	public FollowTrajCommand(Drivetrain dt)
	{
		TrajectoryConfig config =
			new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
								Constants.kMaxAccelerationMetersPerSecondSquared)
				.setKinematics(Drivetrain.kinematics)
				.setStartVelocity(0.2)
				.setEndVelocity(0.0);

		var exampleTrajectory = TrajectoryGenerator.generateTrajectory
		(
			new Pose2d(0, 0, new Rotation2d(0)),
			
			List.of
			(
				new Translation2d(1, 0),
				new Translation2d(1, 1),
				new Translation2d(0, 1)
			),

			new Pose2d(0, 0, new Rotation2d(0)),
			config
		);

		PurePursuitCommand purePursuitCommand = new PurePursuitCommand
		(
			exampleTrajectory, dt::getPose, Drivetrain.kinematics.trackWidthMeters, dt::setWheelSpeeds, Constants.kLookAheadDistance, dt
		);

		dt.resetPose(exampleTrajectory.getInitialPose());

		addCommands(purePursuitCommand.andThen(() -> dt.setWheelSpeeds(0, 0)));
	}
}

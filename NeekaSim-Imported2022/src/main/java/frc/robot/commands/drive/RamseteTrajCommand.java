package frc.robot.commands.drive;

import java.util.List;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
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

		Trajectory traj = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(),
				new Pose2d(3, 3, Rotation2d.fromDegrees(0))
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

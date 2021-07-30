package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TrajectoryCommand extends SequentialCommandGroup
{
    public TrajectoryCommand(Drivetrain m_robotDrive)
    {
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                                Constants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Drivetrain.kinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(1, 1),
                new Translation2d(2, -1)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config
        );

        RamseteCommand ramseteCommand = new RamseteCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            new RamseteController(),
            Drivetrain.kinematics,
            m_robotDrive::setWheelSpeeds,
            m_robotDrive);    
        
        addCommands(ramseteCommand.andThen(() -> m_robotDrive.setWheelSpeeds(0, 0)));
    }
}

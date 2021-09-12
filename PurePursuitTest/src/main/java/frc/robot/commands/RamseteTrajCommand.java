package frc.robot.commands;

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
    public RamseteTrajCommand(Drivetrain m_drivetrain)
    {
        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.ksVolts, 
                                        Constants.kvVoltSecondsPerMeter, 
                                        Constants.kaVoltSecondsSquaredPerMeter),
                Drivetrain.kinematics,
                10);

        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                                Constants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Drivetrain.kinematics)
                .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(0.15, 0)
                // new Translation2d(1, 1),
                // new Translation2d(2, -1)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(.3, 0, new Rotation2d(0)),
            // Pass config
            config
        );

        RamseteCommand ramseteCommand = new RamseteCommand(
            exampleTrajectory,
            m_drivetrain::getPose,
            new RamseteController(),
            new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
            Drivetrain.kinematics,
            m_drivetrain::getWheelSpeeds,
            new PIDController(Constants.kPRamsete, 0, 0),
            new PIDController(Constants.kPRamsete, 0, 0),
            m_drivetrain::tankDriveVolts,
            m_drivetrain);
        
        addCommands(ramseteCommand.andThen(() -> m_drivetrain.setWheelSpeeds(0, 0)));
    }
}

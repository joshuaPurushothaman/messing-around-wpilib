package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TrajectoryTEST extends SequentialCommandGroup
{
    public TrajectoryTEST(Drivetrain dt)
    {
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                                Constants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Drivetrain.kinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                // new Translation2d(0.1, 0.1),
                // new Translation2d(0.2, -0.1)
            ),
            new Pose2d(0.3, 0, new Rotation2d(0)),
            config
        );

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, 
            dt::getPose, new RamseteController(),
            new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
            Drivetrain.kinematics, dt::getWheelSpeeds, 
            new PIDController(Constants.kPDriveVel, 0, 0), new PIDController(Constants.kPDriveVel, 0, 0),
            dt::tankDriveVolts, dt);
        
        

        addCommands
        (
            new InstantCommand(() -> dt.resetSensors()),
            new PrintCommand("Traj started!!"), 
            ramseteCommand,
            new InstantCommand(() -> dt.arcadeDrive(0, 0)),
            new PrintCommand("Traj done!!")
        );
    }
}

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
    public RamseteTrajCommand(Drivetrain dt)
    {       
        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.rksVolts, 
                                            Constants.rkvVoltSecondsPerMeter),
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
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(0.3, 0)
            ),
            new Pose2d(.6, 0, new Rotation2d(0)),
            config
        );

        RamseteCommand ramseteCommand = new RamseteCommand(
            exampleTrajectory,
            dt::getPose,
            new RamseteController(),
            Drivetrain.kinematics,
            (left, right) -> dt.setWheelSpeeds(left, right),
            dt
        );
        
        addCommands
        (
            new InstantCommand(() -> {dt.resetPose(); dt.resetSensors();}),
            ramseteCommand,
            new RunCommand(() -> dt.setWheelSpeeds(0, 0))
        );
    }
}

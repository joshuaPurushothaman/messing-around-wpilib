package frc.robot.commands.PurePursuit;

import java.util.function.*;

import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;

public class PurePursuitCommand extends CommandBase
{
    PurePursuitController controller;
    BiConsumer<Double, Double> outputMetersPerSecond;

    public PurePursuitCommand(Trajectory traj, Supplier<Pose2d> poseSupplier, double trackWidthMeters,
        BiConsumer<Double, Double> outputMetersPerSecond,
        double lookAheadDistanceMeters, Drivetrain dt)
    {
        addRequirements(dt);
        controller = new PurePursuitController(traj, poseSupplier, trackWidthMeters, outputMetersPerSecond, lookAheadDistanceMeters);
        this.outputMetersPerSecond = outputMetersPerSecond;
    }

    @Override
    public void execute()
    {
        controller.setWheelSpeeds();
    }

    @Override
    public void end(boolean interrupted)
    {
        System.out.println(interrupted ? " traj interrupted" : "traj COMPLETED! :D");
    }

    @Override
    public boolean isFinished()
    {
        return controller.isFinished();
    }

    public PurePursuitController getController()
    {
        return controller;
    }
}

package frc.robot.commands.PurePursuit;


import java.util.List;
import java.util.function.BiConsumer;

import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class PPCTesterCommand extends CommandBase
{
    PurePursuitController controller;
    BiConsumer<Double, Double> outputMetersPerSecond;

    public PPCTesterCommand(Drivetrain dt)
    {
        addRequirements(dt);
        
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
				new Translation2d(0.5, 0)
			),

			new Pose2d(0.5, 0.5, new Rotation2d(0)),
			config
		);
        controller = new PurePursuitController(exampleTrajectory, dt::getPose, Drivetrain.kinematics.trackWidthMeters, dt::setWheelSpeeds, Constants.kLookAheadDistance);
        this.outputMetersPerSecond = dt::setWheelSpeeds;
    }

    @Override
    public void execute()
    {
        // controller.setWheelSpeeds();
        // controller.dotProduct()
        // System.out.println(controller.findLookAheadPoint());
    }

    @Override
    public void end(boolean interrupted)
    {
        System.out.println(interrupted ? " ppc test interrupted" : "ppc test COMPLETED! :D");
    }

    @Override
    public boolean isFinished()
    {
        return controller.isFinished();
    }
}

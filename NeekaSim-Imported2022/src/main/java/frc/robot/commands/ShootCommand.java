package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends PIDCommand
{
	public ShootCommand(double velocityRPM, Shooter shooter)
	{
		super(new PIDController(1, 0, 0),
			() -> shooter.getVelocityRPM(),
			velocityRPM, shooter::set, shooter);
	}
}
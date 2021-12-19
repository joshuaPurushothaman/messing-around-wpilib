package frc.robot.commands.aim;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class SimpleAimCommand extends CommandBase
{
	Vision2 vision;
	Drivetrain dt;
	LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);	
	
	private static final double kTurnSpeed = 0.45;

	public SimpleAimCommand(Drivetrain dt, Vision2 vision)
	{
		this.dt = dt;
		this.vision = vision;
	}
	

	@Override
	public void execute()
	{
		double x = filter.calculate(vision.getX());

		if (x > 0)
			dt.arcadeDrive(0, kTurnSpeed);
		else
			dt.arcadeDrive(0, -kTurnSpeed);
	}

	@Override
	public void end(boolean interrupted)
	{
		dt.arcadeDrive(0, 0);
	}
	
	@Override
	public boolean isFinished()
	{
		return Math.abs(vision.getX()) < Constants.kToleranceDegreesAim || !vision.getValid();
	}
}

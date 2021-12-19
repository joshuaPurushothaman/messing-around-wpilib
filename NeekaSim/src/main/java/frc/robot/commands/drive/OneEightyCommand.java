package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class OneEightyCommand extends PIDCommand
{
	Drivetrain dt;
	
    public OneEightyCommand(Drivetrain dt)
    {
        super(new PIDController(Constants.kPTTA, 0, 0),
            dt::getHeading,
            180,
            output -> dt.arcadeDrive(0, output),
            dt);

        this.dt = dt;
        
        getController().setTolerance(Constants.kToleranceDegrees);
        getController().enableContinuousInput(-180, 180);
    }

    @Override
    public void initialize()
    {
        super.initialize();

        double setpoint = dt.getHeading() + 180;
        this.m_setpoint = () -> setpoint;
    }

    @Override
    public boolean isFinished()
    {
        return getController().atSetpoint();
    }
}
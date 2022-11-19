package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TurnToAngleCommand extends PIDCommand
{
    Drivetrain dt;
    double targetAngleDegrees;

    public TurnToAngleCommand(double targetAngleDegrees, Drivetrain dt)
    {
        super(new PIDController(Constants.kPTTA, 0, 0),
            dt::getHeading,
            targetAngleDegrees,
            output -> dt.arcadeDrive(0, output),
            dt);

        this.dt = dt;
        this.targetAngleDegrees = targetAngleDegrees;
        
        getController().setTolerance(Constants.kToleranceDegrees);
        getController().enableContinuousInput(-180, 180);
    }

    @Override
    public void initialize()
    {
        super.initialize();

        double setpoint = dt.getHeading() + targetAngleDegrees;
        this.m_setpoint = () -> setpoint;
    }

    @Override
    public boolean isFinished()
    {
        return getController().atSetpoint();
    }
}
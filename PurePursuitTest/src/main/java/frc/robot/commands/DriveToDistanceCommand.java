package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

public class DriveToDistanceCommand extends PIDCommand
{
    Drivetrain dt;

    public DriveToDistanceCommand(double targetDistanceMeters, Drivetrain dt)
    {
        super(new PIDController(6, 0, 0),
            dt::getAverageDistanceMeters,
            targetDistanceMeters,
            output -> dt.arcadeDrive(output, 0), 
            dt);

        this.dt = dt;

        getController().setTolerance(0.05);

        addRequirements(dt);
    }

    @Override
    public void initialize()
    {
        dt.resetEncoders();
    }

    @Override
    public boolean isFinished()
    {
        return getController().atSetpoint();
    }
    
}

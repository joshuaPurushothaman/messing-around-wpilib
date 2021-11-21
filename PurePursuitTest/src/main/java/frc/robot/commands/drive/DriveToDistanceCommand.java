package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveToDistanceCommand extends PIDCommand
{
    Drivetrain dt;

    public DriveToDistanceCommand(double targetDistanceMeters, Drivetrain dt)
    {
        super(new PIDController(Constants.kPDTD, 0, 0),
            dt::getAverageDistanceMeters,
            targetDistanceMeters,
            output -> dt.arcadeDrive(output, 0), 
            dt);

        this.dt = dt;

        getController().setTolerance(Constants.kToleranceMeters, 0);
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

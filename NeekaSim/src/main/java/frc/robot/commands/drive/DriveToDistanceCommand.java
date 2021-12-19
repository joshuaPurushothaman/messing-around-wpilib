package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveToDistanceCommand extends PIDCommand
{
    Drivetrain dt;
    double distanceMeters;

    public DriveToDistanceCommand(double distanceMeters, Drivetrain dt)
    {
        super(new PIDController(Constants.kPDTD, 0, 0),
            dt::getAverageDistanceMeters,
            distanceMeters,
            output -> dt.arcadeDrive(output, 0), 
            dt);

        this.dt = dt;
        this.distanceMeters = distanceMeters;

        getController().setTolerance(Constants.kToleranceMeters);
    }

    @Override
    public void initialize()
    {
        super.initialize();
        
        double setpoint = distanceMeters + dt.getAverageDistanceMeters();
        this.m_setpoint = () -> setpoint;
    }

    @Override
    public boolean isFinished()
    {
        return getController().atSetpoint();
    }
}

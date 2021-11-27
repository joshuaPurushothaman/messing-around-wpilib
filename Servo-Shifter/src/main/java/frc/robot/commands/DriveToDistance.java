package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.drive.Drivetrain;

public class DriveToDistance extends PIDCommand
{
    Drivetrain dt;

    public DriveToDistance(double distanceMeters, Drivetrain dt)
    {
        super(new PIDController(2.5, 0, 0),
            () -> dt.getDistance(),
            distanceMeters,
            (output) -> dt.arcadeDrive(output, 0),
            dt);

        this.dt = dt;
    }

    @Override
    public void initialize()
    {
        dt.setAutomaticShifting(false);
        dt.shift(false);
    }

    @Override
    public void end(boolean interrupted)
    {
        dt.setAutomaticShifting(true);
    }
}

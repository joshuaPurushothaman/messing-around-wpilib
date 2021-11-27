package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class UltimateDTD extends CommandBase
{
    Drivetrain dt;
    ProfiledPIDController distController = new ProfiledPIDController(Constants.kPDTD, 0, 0, 
        new TrapezoidProfile.Constraints(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared));
    PIDController angleController = new PIDController(Constants.kPTTA, 0, 0);

    public UltimateDTD(double distanceMeters, Drivetrain dt)
    {
        this.dt = dt;

        distController.setTolerance(Constants.kToleranceMeters);
        distController.setGoal(distanceMeters);

        angleController.setTolerance(Constants.kToleranceDegrees);
        angleController.setSetpoint(0);
    }
    
    @Override
    public void initialize()
    {
        dt.resetSensors();
    }

    @Override
    public void execute()
    {
        dt.arcadeDrive(distController.calculate(dt.getAverageDistanceMeters()), 
            angleController.calculate(dt.getHeading(), 0));
    }

    @Override
    public void end(boolean interrupted)
    {
        dt.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished()
    {
        return distController.atGoal() && angleController.atSetpoint();
    }
}
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.*;

public class SwerveDriveCommand extends CommandBase
{
    Swerve swerve;

    public SwerveDriveCommand(Swerve swerve)
    {
        this.swerve = swerve;
    }
    
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }
    
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
}

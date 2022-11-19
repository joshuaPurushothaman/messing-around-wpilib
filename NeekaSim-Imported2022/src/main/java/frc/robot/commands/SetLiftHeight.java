package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Lift;

public class SetLiftHeight extends ProfiledPIDCommand
{
	Lift lift;
    double positionMeters;

    public SetLiftHeight(double positionMeters, Lift lift)
    {
        super(new ProfiledPIDController(Constants.kPLift, 0, 0, 
                new TrapezoidProfile.Constraints(Constants.kMaxSpeedMetersPerSecondLift, 
                    Constants.kMaxAccelerationMetersPerSecondSquaredLift)), 
            () -> lift.getPositionMeters(),
            new TrapezoidProfile.State(positionMeters, 0),
            (output, setpoint) -> lift.set(output),
            lift);
        
        this.lift = lift;
        this.positionMeters = positionMeters;

        getController().setTolerance(Constants.kToleranceMetersLift);
    }
    
    
    @Override
    public void initialize()
    {
        super.initialize();

        var goal = new TrapezoidProfile.State(positionMeters + lift.getPositionMeters(), 0);
        this.m_goal = () -> goal;
    }

    @Override
    public boolean isFinished()
    {
        return getController().atGoal();
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class AimCommand extends ProfiledPIDCommand
{
    Vision vision;
    Drivetrain dt;

    public AimCommand(Drivetrain dt, Vision vision)
    {
        super(new ProfiledPIDController(Constants.kPAim, 0, 0,
                new TrapezoidProfile.Constraints(Constants.kMaxTurnSpeedDegreesPerSecondAim, 
                    Constants.kMaxTurnAccelerationDegreesPerSecondSquaredAim)),
            () -> -vision.getX(), 
            new TrapezoidProfile.State(0, 0),
            (output, setpoint) -> dt.arcadeDrive(0, output),
            dt, vision);

            this.vision = vision;
            this.dt = dt;

            dt.setMaxOutput(0.25);

            getController().setTolerance(Constants.kToleranceDegreesAim);
    }

    @Override
    public void end(boolean interrupted)
    {
        super.end(interrupted);

        dt.setMaxOutput(1);
    }

    @Override
    public boolean isFinished()
    {
        return getController().atGoal() || !vision.getV();
    }
}

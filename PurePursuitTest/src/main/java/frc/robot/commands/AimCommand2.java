package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class AimCommand2 extends ProfiledPIDCommand
{
    Vision vision;
    Drivetrain dt;

    public AimCommand2(Drivetrain dt, Vision vision)
    {
        super(new ProfiledPIDController(Constants.kPTTA, 0, 0,
                new TrapezoidProfile.Constraints(Constants.kMaxTurnSpeedDegreesPerSecond, 
                    Constants.kMaxTurnAccelerationDegreesPerSecondSquared)),
            dt::getHeading,
            new TrapezoidProfile.State(-vision.getX(), 0),
            (output, setpoint) -> dt.arcadeDrive(0, output),
            dt, vision);

            this.vision = vision;
            this.dt = dt;

            getController().setTolerance(Constants.kToleranceDegreesAim);
    }

    @Override
    public boolean isFinished()
    {
        return getController().atGoal() || !vision.getV();
    }
}

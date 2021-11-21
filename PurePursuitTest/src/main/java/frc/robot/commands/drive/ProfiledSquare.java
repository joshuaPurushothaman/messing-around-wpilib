package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class ProfiledSquare extends SequentialCommandGroup
{
    public ProfiledSquare(Drivetrain dt)
    {
        addCommands
        (
            new DTDProfiled(0.3, dt),
            new TTAProfiled(90, dt),
            new DTDProfiled(0.3, dt),
            new TTAProfiled(90, dt),
            new DTDProfiled(0.3, dt),
            new TTAProfiled(90, dt),
            new DTDProfiled(0.3, dt),
            new TTAProfiled(90, dt),
            new DTDProfiled(0.3, dt),
            new TTAProfiled(90, dt)
        );
    }
}

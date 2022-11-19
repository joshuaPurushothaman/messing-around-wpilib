package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class Square extends SequentialCommandGroup
{
    public Square(Drivetrain dt)
    {
        addCommands
        (
            new DriveToDistanceCommand(1, dt),
            new TurnToAngleCommand(90, dt),
            new DriveToDistanceCommand(1, dt),
            new TurnToAngleCommand(90, dt),
            new DriveToDistanceCommand(1, dt),
            new TurnToAngleCommand(90, dt),
            new DriveToDistanceCommand(1, dt),
            new TurnToAngleCommand(90, dt)
        );
    }
}

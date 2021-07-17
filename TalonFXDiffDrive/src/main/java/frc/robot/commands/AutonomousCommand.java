package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveToDistanceCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.subsystems.*;
public class AutonomousCommand extends SequentialCommandGroup
{
    public AutonomousCommand(Drivetrain dt)
    {
        addCommands
        (
            //  Drives in a 1x1 meter square, by executing the following commands in sequence.
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

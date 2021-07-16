package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DiffDriveTraj;

public class AutoCommand extends SequentialCommandGroup
{
    public AutoCommand(DiffDriveTraj dt)
    {
        Trajectory traj = null; //  TODO: SETUP PATHWEAVRERRE
        
        addCommands
        (
            new RamseteCommand(
                traj, dt::getPose, new RamseteController(), dt.getKinematics(), dt::setWheelSpeeds, dt
            )
        );
    }
}

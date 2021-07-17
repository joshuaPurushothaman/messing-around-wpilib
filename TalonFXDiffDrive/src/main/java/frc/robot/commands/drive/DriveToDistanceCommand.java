package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

public class DriveToDistanceCommand extends PIDCommand
{
    Drivetrain dt;
    
    public DriveToDistanceCommand(double distMeters, Drivetrain dt)
    {
         //  TODO: tune PID constants, this should work for now though
        super(new PIDController(2.5, 0, 0),
            dt::getDistance,    //  PID's process variable getter method
            distMeters, //  PID setpoint
            (output) -> dt.drive(output, 0),    //  PID output method as a lambda, this will turn the robot to the desired angle
            dt);    //  every command must have its subsystems passed in as "requirements"

        getController().setTolerance(0.1);  //  the tolerance with which the isFinished() method checks if the PV is within the setpoint

        this.dt = dt;
    }

    @Override
    public void initialize()
    {
        dt.resetEncoders(); //  upon initialization, reset the encoders
    }

    @Override
    public boolean isFinished()
    {
        return getController().atSetpoint();    //  This command will terminate once the desired distance has been reached.
    }
}

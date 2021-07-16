package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import frc.robot.Constants;

public class DiffDriveTraj extends DiffDriveOdom
{
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TRACK_WIDTH);
    
    CANPIDController lfPID = lf.getPIDController(),
                        rfPID = rf.getPIDController();
    
    public DiffDriveTraj()
    {
        super();
        
        lfPID.setP(1);
        rfPID.setP(1);  //  TODO: tune, put PIDs in Constants

        lb.follow(lf);
        rb.follow(rf);
    }
    public DifferentialDriveKinematics getKinematics()
    {
        return kinematics;
    }

    public void setWheelSpeeds(double leftSpeed, double rightSpeed)
    {
        lfPID.setReference(leftSpeed, ControlType.kVelocity);
        rfPID.setReference(rightSpeed, ControlType.kVelocity);
    }
}

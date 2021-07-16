package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase
{
    WPI_TalonSRX leftMaster = new WPI_TalonSRX(2);
    WPI_TalonSRX leftSlave = new WPI_TalonSRX(1);
    WPI_TalonSRX rightMaster = new WPI_TalonSRX(3);
    WPI_TalonSRX rightSlave = new WPI_TalonSRX(4);

    DifferentialDrive dt = new DifferentialDrive(
        new SpeedControllerGroup(leftMaster, leftSlave), 
        new SpeedControllerGroup(rightMaster, rightSlave));

    public Drivetrain()
    {
        leftMaster.setInverted(true);
        leftSlave.setInverted(true);
        rightMaster.setInverted(false);
        rightSlave.setInverted(false);
    }

    public void arcadeDrive(double xSpeed, double zRotation, boolean slowMode)
    {
        dt.setMaxOutput(slowMode ? 0.3 : 1);
        dt.arcadeDrive(xSpeed, zRotation);
    }
}

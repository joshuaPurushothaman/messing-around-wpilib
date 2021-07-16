package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase
{
    CANSparkMax lf = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax lb = new CANSparkMax(1, MotorType.kBrushless);
    CANSparkMax rf = new CANSparkMax(2, MotorType.kBrushless);
    CANSparkMax rb = new CANSparkMax(3, MotorType.kBrushless);

    DifferentialDrive dt = new DifferentialDrive
    (
        new SpeedControllerGroup(lf, lb),
        new SpeedControllerGroup(rf, rb)
    );

    public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn)
    {
        dt.curvatureDrive(xSpeed, zRotation, isQuickTurn);
    }
}

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.*;
import frc.robot.Constants;

public class DiffDriveOdom extends DiffDrive
{
    ADXRS450_Gyro gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
    CANEncoder lfEncoder = lf.getEncoder(),
                rfEncoder = rf.getEncoder();

    DifferentialDriveOdometry odom = new DifferentialDriveOdometry(new Rotation2d(-gyro.getAngle()));

    public DiffDriveOdom()
    {
        super();
        lfEncoder.setPositionConversionFactor(1/42 * Constants.Drivetrain.WHEEL_DIAMETER_METERS);
        rfEncoder.setPositionConversionFactor(1/42 * Constants.Drivetrain.WHEEL_DIAMETER_METERS);
        
        lfEncoder.setVelocityConversionFactor(1/42 * Constants.Drivetrain.WHEEL_DIAMETER_METERS);
        rfEncoder.setVelocityConversionFactor(1/42 * Constants.Drivetrain.WHEEL_DIAMETER_METERS);
    }

    /**
     * note: will reset odometry
     */
    public void resetSensorsAndOdoms()
    {
        lfEncoder.setPosition(0);
        rfEncoder.setPosition(0);
        
        gyro.reset();
        gyro.calibrate();

        odom.resetPosition(new Pose2d(), new Rotation2d());
    }

    public Pose2d getPose()
    {
        return odom.getPoseMeters();
    }

    @Override
    public void periodic()
    {
        super.periodic();
    
        odom.update(new Rotation2d(-gyro.getAngle()), lfEncoder.getPosition(), rfEncoder.getPosition());
    }
}

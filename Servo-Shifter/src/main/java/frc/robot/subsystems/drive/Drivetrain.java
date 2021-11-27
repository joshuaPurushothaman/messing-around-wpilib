package frc.robot.subsystems.drive;

import com.revrobotics.*;
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
    
    CANEncoder lfEncoder = lf.getEncoder();
    CANEncoder rfEncoder = rf.getEncoder();

    DifferentialDrive dt = new DifferentialDrive
    (
        new SpeedControllerGroup(lf, lb),
        new SpeedControllerGroup(rf, rb)
    );

    LinearServo leftShifter = new LinearServo(0, 50, 32);
    LinearServo rightShifter = new LinearServo(1, 50, 32);

    // TODO: Example Positions
    private static final double highGearPositionMm = 40;
    private static final double lowGearPositionMm = 10;

    boolean automaticShiftingEnabled = true;


    public void arcadeDrive(double xSpeed, double zRotation)
    {
        dt.arcadeDrive(xSpeed, zRotation);
    }
    
    public double getDistance()
    {
		return (lfEncoder.getPosition() + rfEncoder.getPosition()) / 2;
	}
    
    public void setAutomaticShifting(boolean isAutomaticShiftingEnabled)
    {
        automaticShiftingEnabled = isAutomaticShiftingEnabled;
    }

    /**
     * Should be called repeatedly so that the servo can reach the setpoint.
     * @param isHighGear high if true, low if false
     */
    public void shift(boolean isHighGear)
    {
        if (isHighGear) {
            leftShifter.setPosition(highGearPositionMm);
            rightShifter.setPosition(highGearPositionMm);
            
            lfEncoder.setPositionConversionFactor(69420);
            lfEncoder.setVelocityConversionFactor(69420);
            rfEncoder.setPositionConversionFactor(69420);
            rfEncoder.setVelocityConversionFactor(69420);
        } else {
            leftShifter.setPosition(lowGearPositionMm);
            rightShifter.setPosition(lowGearPositionMm);

            lfEncoder.setPositionConversionFactor(24960);
            lfEncoder.setVelocityConversionFactor(24960);
            rfEncoder.setPositionConversionFactor(24960);
            rfEncoder.setVelocityConversionFactor(24960);
        }
    }

    @Override
    public void periodic()
    {
        final double shiftVel = 10;
        
        if (automaticShiftingEnabled == true)
            shift(Math.abs(lfEncoder.getVelocity() + rfEncoder.getVelocity()) > shiftVel);
    }
}

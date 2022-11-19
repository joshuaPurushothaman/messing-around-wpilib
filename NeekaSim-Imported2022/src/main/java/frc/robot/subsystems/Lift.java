package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * GreyT Elevator v2
 */
public class Lift extends SubsystemBase
{
	CANSparkMax liftMotor = new CANSparkMax(Constants.LIFT_MOTOR_ID, MotorType.kBrushless);

	RelativeEncoder liftEncoder = liftMotor.getEncoder();

	private static final double kElevatorDrumRadius = Units.inchesToMeters(1.76) / 2;
	private static final double kCountsPerRevolution = 360;
	
	public static final double HIGH_POSITION_METERS = Units.feetToMeters(6);
	public static final double LOW_POSITION_METERS = Units.feetToMeters(0);

	public Lift()
	{
		liftEncoder.setPositionConversionFactor(2.0 * Math.PI * kElevatorDrumRadius / kCountsPerRevolution);
		liftEncoder.setVelocityConversionFactor(2.0 * Math.PI * kElevatorDrumRadius / kCountsPerRevolution);

		liftMotor.burnFlash();
	}

	public void set(double power)
	{
		liftMotor.set(power);
	}
	
	public double getPositionMeters()
	{
		return liftEncoder.getPosition();
	}

	@Override
	public void periodic()
	{
		SmartDashboard.putNumber("Lift Height", getPositionMeters());
	}
}
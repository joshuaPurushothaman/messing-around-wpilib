package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase
{
	public static final double SHOOT_VELOCITY_RPM = 1000;

	CANSparkMax leftMotor = new CANSparkMax(Constants.LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
	CANSparkMax rightMotor = new CANSparkMax(Constants.RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);

	// RelativeEncoder encoder = leftMotor.getEncoder();
	Encoder encoder = new Encoder(29, 30);

	public Shooter()
	{
		leftMotor.setInverted(false);
		rightMotor.setInverted(true);

		leftMotor.burnFlash();
		rightMotor.burnFlash();
	}

	public void set(double power)
	{
		leftMotor.set(power);
		rightMotor.set(power);
	}

	public double getVelocityRPM()
	{
		// return encoder.getVelocity();
		return encoder.getRate();
	}

	@Override
	public void periodic()
	{
		SmartDashboard.putNumber("Shooter Velocity", getVelocityRPM());
	}
}

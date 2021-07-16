package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DiffDrive extends SubsystemBase
{
	CANSparkMax lf = new CANSparkMax(Constants.Drivetrain.lf, MotorType.kBrushless),
				rf = new CANSparkMax(Constants.Drivetrain.rf, MotorType.kBrushless),
				lb = new CANSparkMax(Constants.Drivetrain.lb, MotorType.kBrushless),
				rb = new CANSparkMax(Constants.Drivetrain.rb, MotorType.kBrushless);
	
	DifferentialDrive dt = new DifferentialDrive(new SpeedControllerGroup(lf, lb), new SpeedControllerGroup(rf, rb));
	
	public void arcadeDrive(double xSpeed, double zRotation)
	{
		dt.arcadeDrive(xSpeed, zRotation);
	}

	public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn)
	{
		dt.curvatureDrive(xSpeed, zRotation, isQuickTurn);
	}
	
	public void tankDrive(double leftSpeed, double rightSpeed)
	{
		dt.tankDrive(leftSpeed, rightSpeed);
	}

	public void setMaxOutput(double maxOutput)
	{
		dt.setMaxOutput(maxOutput);
	}

	public void setSlowMode(boolean isSlowMode)
	{
		if (isSlowMode)
			setMaxOutput(Constants.Drivetrain.SLOW_MODE_SPEED);
		else
			setMaxOutput(Constants.Drivetrain.NON_SLOW_MODE_SPEED);
	}
}

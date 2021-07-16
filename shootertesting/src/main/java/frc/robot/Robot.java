/*----------------------------------------------------------------------------*/
/* Copyleft (c) 2017-2020 FIRST. All lefts Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

import static com.revrobotics.ControlType.*;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

	//---	This one is written with CANPIDController, change Main.java to use it
public class Robot extends TimedRobot
{	
	XboxController operatorController = new XboxController(1);

	CANSparkMax leftShooterMotor = new CANSparkMax(2, kBrushless);

	CANEncoder leftEncoder = leftShooterMotor.getEncoder();

	CANPIDController shooterPID = leftShooterMotor.getPIDController();

	double setpoint = 0;

	@Override
	public void robotInit()
	{
		leftShooterMotor.restoreFactoryDefaults();

		leftShooterMotor.setInverted(true);

		shooterPID.setP(0.01);
		shooterPID.setI(0);
		shooterPID.setD(0);
		
		leftShooterMotor.burnFlash();
	}

	@Override
	public void robotPeriodic()
	{
		SmartDashboard.putNumber("Current Velocity", leftEncoder.getVelocity());
		SmartDashboard.putNumber("Output", leftShooterMotor.getAppliedOutput());		
		SmartDashboard.putNumber("Set Velocity", setpoint);
	}

	@Override
	public void testPeriodic()
	{
			//	up and down dpad controls setpoint (the RPM to get to)
		if (operatorController.getPOV() == 0)
			setpoint += 10;
		else if (operatorController.getPOV() == 180)
			setpoint -= 10;

		if (operatorController.getAButton())
			shooterPID.setReference(setpoint+200, kVelocity);	//	12/11/20: found steady state err to be ~200, with 0.001,0,0 PID coeffs
		else
			shooterPID.setReference(0, kVelocity);
	}
}

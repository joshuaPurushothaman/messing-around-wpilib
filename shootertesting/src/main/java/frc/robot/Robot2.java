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

/**
 * Testing Ziegler-Nichols PID tuning method https://youtu.be/UOuRx9Ujsog?t=648
 * 
 * 1. Set kI and kD to 0
 * 2. Increase kP until output is a "sustained and stable oscillation"
 * 3. Record the Critical Gain kC (the kP that caused the oscillation), and the oscillation period pC
 * 4. Set gains:
 * 	kP = 0.6*kC
 * 	kI = 2*kP/pC
 * 	kD = 0.125*kP*Pc
 */
public class Robot2 extends TimedRobot
{	
	XboxController operatorController = new XboxController(1);

	CANSparkMax leftShooterMotor = new CANSparkMax(6, kBrushless);
	CANEncoder leftEncoder = leftShooterMotor.getEncoder();
	CANPIDController shooterPID = leftShooterMotor.getPIDController();
	
	final double RPM_10FTLINE = 1350;

	@Override
	public void robotInit()
	{
		leftShooterMotor.restoreFactoryDefaults();		
		leftShooterMotor.setInverted(true);
	}

	@Override
	public void robotPeriodic()
	{
		SmartDashboard.putNumber("Current Velocity", leftEncoder.getVelocity());
		SmartDashboard.putNumber("Output", leftShooterMotor.getAppliedOutput());
		
		shooterPID.setP(SmartDashboard.getNumber("kP", 0.001));
		shooterPID.setI(SmartDashboard.getNumber("kI", 0));
		shooterPID.setD(SmartDashboard.getNumber("kD", 0));
	}

	@Override
	public void testPeriodic()
	{
		leftShooterMotor.set(0.5);
		// 	//	12/11/20: found steady state err to be ~200, with 0.001, 0, 0 PID coeffs
		// if (operatorController.getAButton())
		// 	shooterPID.setReference(RPM_10FTLINE, kVelocity);
		// else
		// 	shooterPID.setReference(0, kVelocity);
	}
}

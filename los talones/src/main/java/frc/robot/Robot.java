// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot
{
	WPI_TalonFX left = new WPI_TalonFX(0);
	WPI_TalonFX right = new WPI_TalonFX(1);

	@Override
	public void robotInit()
	{
		right.setInverted(true);
		left.setNeutralMode(NeutralMode.Coast);
		right.setNeutralMode(NeutralMode.Coast);
	}

	@Override
	public void robotPeriodic() {}

	@Override
	public void testInit() {}

	@Override
	public void testPeriodic() 
	{
		left.set(ControlMode.PercentOutput, -1);
		right.set(ControlMode.PercentOutput, 1);
	}
}

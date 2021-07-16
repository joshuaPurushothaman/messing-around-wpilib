/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.wpilibj.GenericHID.Hand.*;

public class Robot extends TimedRobot
{
	Mecanum dt = new Mecanum();
	XboxController driverController = new XboxController(0);
	Skillz skills = new Skillz(dt);

	@Override
	public void robotInit()
	{
		dt.resetSensors();
	}

	@Override
	public void robotPeriodic()
	{
		dt.displayEncoders();
		dt.displayGyro();
		
		angle = SmartDashboard.getNumber("Turn To?", 0);
		SmartDashboard.putNumber("Turn To?", angle);
	}

	double angle;

	@Override
	public void teleopInit()
	{
		dt.resetSensors();
	}

	@Override
	public void teleopPeriodic()
	{
		dt.drive(driverController.getX(kLeft), -driverController.getY(kLeft),
				driverController.getX(kRight));
	}

	@Override
	public void testInit()
	{
		skills.slalomPath();
		// skills.selectSimplePath();
		skills.init();
	}

	@Override
	public void testPeriodic()
	{
		skills.periodic();
		
		
		// if (driverController.getAButton())
		// 	dt.turnToAngle(angle);
		// else
		// 	dt.drive(0, 0, 0);
	}        
}

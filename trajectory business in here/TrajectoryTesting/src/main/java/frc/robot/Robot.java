/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import static edu.wpi.first.wpilibj.GenericHID.Hand.*;

import frc.robot.subsystems.*;

public class Robot extends TimedRobot
{
	EzMecNODOM dt = new EzMecNODOM();
	XboxController driverController = new XboxController(0);
	Timer autoTimer = new Timer();

	@Override
	public void autonomousInit()
	{
		dt.calibrate();
		autoTimer.reset();
		autoTimer.start();
	}

	@Override
	public void autonomousPeriodic()
	{
		if (autoTimer.get() <= 5)
			dt.driveToDistance(5);
		else if (autoTimer.get() <= 10)
			dt.turnToAngle(90);
		else if (autoTimer.get() <= 15)
			dt.driveToDistance(2);
	}

	@Override
	public void teleopPeriodic()
	{
		if (!driverController.getAButton())
			dt.drive(driverController.getY(kLeft), driverController.getX(kLeft), driverController.getX(kRight), false);
		else
			dt.aim();
	}
}

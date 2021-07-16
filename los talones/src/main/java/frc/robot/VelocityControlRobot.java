// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class VelocityControlRobot extends TimedRobot
{
	Shooter420 Jose_Ramirez = new Shooter420();	//	my good friend jose... hola taco beans
	XboxController driverController = new XboxController(0);

	@Override
	public void robotInit()	{}

	@Override
	public void robotPeriodic() { Jose_Ramirez.printSpeed(); }

	@Override
	public void testInit() {}

	@Override
	public void testPeriodic() 
	{
		if (driverController.getAButton())
			Jose_Ramirez.shoot();
		else
			Jose_Ramirez.stop();
	// thanks jose! adios amigo :) :) :)
	}
}
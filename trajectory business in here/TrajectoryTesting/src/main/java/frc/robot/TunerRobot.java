/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.wpilibj.GenericHID.Hand.*;

import frc.robot.subsystems.*;

public class TunerRobot extends TimedRobot
{
	EzMecNODOM dt = new EzMecNODOM();
	XboxController driverController = new XboxController(0);

	Pose2d robotPose = new Pose2d();

	@Override
	public void teleopPeriodic()
	{
		robotPose.transformBy(new Transform2d(
			new Translation2d(0, driverController.getY(kLeft)), // arcade drive-y
			new Rotation2d(driverController.getX(kRight))));
		
		SmartDashboard.putString("pose", robotPose.toString());

		if (driverController.getAButton())
			dt.driveToDistance(robotPose.getTranslation().getY());
		else if (driverController.getBButton())
			dt.turnToAngle(robotPose.getRotation().getDegrees());
		else
			dt.drive(0, 0, 0, false);


			/// Different stuff below... gets you from one pose to the other?

		// Pose2d goTo = new Pose2d(10, 10, Rotation2d.fromDegrees(90));	//	90 deg CCW from East... math
		// Pose2d home = new Pose2d();	//	but it's all relative so whatevs
		
		// if (driverController.getAButton())
		// {
		// 	dt.turnToAngle(goTo.minus(home).getRotation().getDegrees());
		// 	/// THEN **** Like, don't turn AND drive at the same time here!
		// 	dt.driveToDistance(goTo.minus(home).getTranslation().getNorm());
		// 	/// THEN **** Like, don't turn AND drive at the same time here!
		// 	dt.turnToAngle(goTo.getRotation().getDegrees());
		// }
	}
}

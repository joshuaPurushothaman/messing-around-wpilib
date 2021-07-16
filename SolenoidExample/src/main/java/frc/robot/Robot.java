// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;

/**
 * This is a sample program showing the use of the solenoid classes during operator control. Three
 * buttons from a joystick will be used to control two solenoids: One button to control the position
 * of a single solenoid and the other two buttons to control a double solenoid. Single solenoids can
 * either be on or off, such that the air diverted through them goes through either one channel or
 * the other. Double solenoids have three states: Off, Forward, and Reverse. Forward and Reverse
 * divert the air through the two channels and correspond to the on and off of a single solenoid,
 * but a double solenoid can also be "off", where the solenoid will remain in its default power off
 * state. Additionally, double solenoids take up two channels on your PCM whereas single solenoids
 * only take a single channel.
 */
public class Robot extends TimedRobot
{
	XboxController controller = new XboxController(0);

	
	// Solenoid corresponds to a single solenoid.
	private final Solenoid m_solenoid = new Solenoid(0);

	// DoubleSolenoid corresponds to a double solenoid.
	private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(1, 2);

	@Override
	public void teleopPeriodic()
	{
		/*
		 * The output of GetRawButton is true/false depending on whether
		 * the button is pressed; Set takes a boolean for whether
		 * to use the default (false) channel or the other (true).
		 */
		m_solenoid.set(controller.getAButton());

		/*
		 * In order to set the double solenoid, if just one button
		 * is pressed, set the solenoid to correspond to that button.
		 * If both are pressed, set the solenoid will be set to Forwards.
		 */
		if (controller.getPOV() == 90) {
			m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
		} else if (controller.getPOV() == 270) {
			m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
		}
	}
}

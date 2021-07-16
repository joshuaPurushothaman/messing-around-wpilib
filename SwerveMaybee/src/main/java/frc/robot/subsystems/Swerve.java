// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;

import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.commands.*;

public class Swerve extends SubsystemBase
{
	public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d());

	SwerveModule[] modules = new SwerveModule[4];
	
	public Swerve()
	{
		setDefaultCommand(new SwerveDriveCommand(this));
	}

	@Override
	public void periodic()
	{
		// This method will be called once per scheduler run
	}

	public Pose2d getPosition()
	{
		return null;
	}

	public void setModuleStates(SwerveModuleState[] moduleStates)
	{
		for (int i = 0; i < 4; i++)
			modules[i].setState(moduleStates[i]);
	}

	public class SwerveModule
	{
		CANSparkMax steer, drive;
		CANPIDController steerPID, drivePID;
		CANEncoder steerEnc, driveEnc;

		public static final double WHEEL_DIAMETER = 0;	//	TODO: wheel diameter
		public Transform2d position;

		public SwerveModule(Transform2d position, CANSparkMax steer, CANSparkMax drive)
		{
			this.position = position;
			this.steer = steer;
			this.drive = drive;

			steerPID = steer.getPIDController();
			drivePID = drive.getPIDController();

			setPID(steerPID, 1, 0, 0);	//	TODO: wat
			setPID(drivePID, 1, 0, 0);	//	TODO: watz

			steerEnc = steer.getEncoder();
			driveEnc = drive.getEncoder();

			steerEnc.setPositionConversionFactor(000);	//	TODO: wat
			driveEnc.setVelocityConversionFactor(000);	//	TODO: watis

			steer.burnFlash();
			drive.burnFlash();
		}

		private void setPID(CANPIDController pid, double kP, double kI, double kD)
		{
			pid.setP(kP);
			pid.setI(kI);
			pid.setD(kD);
		}

		public void setState(SwerveModuleState state)
		{
			setAngle(state.angle.getDegrees());
			setVelocity(state.speedMetersPerSecond);
		}

		public void setAngle(double angle)
		{
			steerPID.setReference(angle, ControlType.kPosition);
		}

		public double getAngle()
		{
			return steerEnc.getPosition();
		}

		public void setVelocity(double velocity)
		{
			drivePID.setReference(velocity, ControlType.kVelocity);
		}

		public double getVelocity()
		{
			return driveEnc.getVelocity();
		}
	}
}

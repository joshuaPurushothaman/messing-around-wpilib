package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


//  links: https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/IntegratedSensor/src/main/java/frc/robot/Robot.java
//  https://docs.revrobotics.com/sparkmax/software-resources/migrating-ctre-to-rev
public class Drivetrain extends SubsystemBase
{
    WPI_TalonFX lf = new WPI_TalonFX(Constants.LF_MOTOR_ID);
    WPI_TalonFX rf = new WPI_TalonFX(Constants.RF_MOTOR_ID);
    WPI_TalonFX lb = new WPI_TalonFX(Constants.LB_MOTOR_ID);
    WPI_TalonFX rb = new WPI_TalonFX(Constants.RB_MOTOR_ID);

    Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

    SpeedControllerGroup leftMotors = new SpeedControllerGroup(lf, lb);
    SpeedControllerGroup rightMotors = new SpeedControllerGroup(rf, rb);

    DifferentialDrive dt = new DifferentialDrive(leftMotors, rightMotors);

	public static final double AUTON_MAX_OUTPUT = 1.0;	//	TODO: tune maxoutputs
    public static final double TELEOP_MAX_OUTPUT = 1.0;
    
    final int kUnitsPerRevolution = 2048;   //  encoder ticks per revolution for talonfx
    final double kMetersPerRotation = Math.PI * Units.feetToMeters(6);   //  how many meters the robot travels with one rotation of the wheels

    public Drivetrain()
    {
        dt.setMaxOutput(AUTON_MAX_OUTPUT);

        //  Setup motor controllers.
        TalonFXConfiguration configs = new TalonFXConfiguration();
        //  Choosing the internal encoder.
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        //  Save to motor controller.
        lf.configAllSettings(configs);
        rf.configAllSettings(configs);
        lb.configAllSettings(configs);
        rb.configAllSettings(configs);

        resetSensors(); //  reset encoders and gyros to 0 position

        rf.setInverted(true);   //  change as necessary
        rb.setInverted(true);

        //  stopping behavior of the motors. Coast will let them slide freely, Brake will freeze them. Try out both.
        final var neutralMode = NeutralMode.Coast;

        lf.setNeutralMode(neutralMode);
        rf.setNeutralMode(neutralMode);
        lb.setNeutralMode(neutralMode);
        rb.setNeutralMode(neutralMode);

        SmartDashboard.putData((Sendable) gyro);    //  sets up gyro for printing angle data as a gyro widget
    }

    /**
	 * Drives the robot.
	 * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
	 * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
	 */
	public void drive(double xSpeed, double zRotation)
	{
		dt.arcadeDrive(xSpeed, zRotation);
	}

	/**
	 * Sets the maximum output power of the drivetrain.
	 * @param maxOutput maximum output power from 0.0 to 1.0
	 */
	public void setMaxOutput(double maxOutput)
	{
		dt.setMaxOutput(maxOutput);
	}

    public double getDistance()
    {
        double lfPosition = kMetersPerRotation * lf.getSelectedSensorPosition() / kUnitsPerRevolution;
        // double rfPosition = kMetersPerRotation * rf.getSelectedSensorPosition() / kUnitsPerRevolution;

        return lfPosition;  //  or average them both, whatever
    }

    /**
	 * Gets the angle that the robot is facing in relative to the last reset / start angle.
	 * @return heading of the robot in degrees
	 */
	public double getAngle()
	{
		return gyro.getAngle();
	}

    public void resetSensors()
    {
        resetEncoders();
        resetGyro();
    }

    public void resetEncoders()
    {
        lf.setSelectedSensorPosition(0);
        rf.setSelectedSensorPosition(0);
    }

    public void resetGyro()
    {
        gyro.reset();
        gyro.calibrate();
    }

    /**
     * prints to SmartDashboard, periodically updating
     */
    @Override
    public void periodic()
    {
        //  by defaylt, talonFX reports position in "ticks", which we here convert to revolutions, then to meters
        double lfPosition = kMetersPerRotation * lf.getSelectedSensorPosition() / kUnitsPerRevolution;
        double rfPosition = kMetersPerRotation * rf.getSelectedSensorPosition() / kUnitsPerRevolution;
        
        SmartDashboard.putNumber("lf position", lfPosition);
		SmartDashboard.putNumber("rf position", rfPosition);


        // by default, talonFX reports velocity in "ticks per 100ms", which we here convert to revolutions per second, then to meters per second
        double lfVelocityRPS = lf.getSelectedSensorVelocity() / kUnitsPerRevolution * 10;
        double rfVelocityRPS = rf.getSelectedSensorVelocity() / kUnitsPerRevolution * 10;
        
        double lfVelocityMetersPerSecond = lfVelocityRPS * kMetersPerRotation;
        double rfVelocityMetersPerSecond = rfVelocityRPS * kMetersPerRotation;

        SmartDashboard.putNumber("lf Velocity", lfVelocityMetersPerSecond);
        SmartDashboard.putNumber("rf Velocity", rfVelocityMetersPerSecond);
    }
}

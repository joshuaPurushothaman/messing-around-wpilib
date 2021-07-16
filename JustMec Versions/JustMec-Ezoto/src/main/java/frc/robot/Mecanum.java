package frc.robot;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.*;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import static com.revrobotics.CANSparkMax.IdleMode.*;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Sendable;

import static edu.wpi.first.wpilibj.SPI.Port.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Mecanum
{
    /// Spitfire BIG
    CANSparkMax lf = new CANSparkMax(1, kBrushless);
    CANSparkMax lb = new CANSparkMax(6, kBrushless);
    CANSparkMax rf = new CANSparkMax(2, kBrushless);
    CANSparkMax rb = new CANSparkMax(5, kBrushless);

    /// Minispit
    // CANSparkMax lf = new CANSparkMax(2, kBrushless);
    // CANSparkMax lb = new CANSparkMax(1, kBrushless);
    // CANSparkMax rf = new CANSparkMax(3, kBrushless);
    // CANSparkMax rb = new CANSparkMax(4, kBrushless);

    MecanumDrive md = new MecanumDrive(lf, lb, rf, rb);

    CANEncoder m_leftFrontEncoder = lf.getEncoder();
    CANEncoder m_leftBackEncoder = lb.getEncoder();
    CANEncoder m_rightFrontEncoder = rf.getEncoder();
    CANEncoder m_rightBackEncoder = rb.getEncoder();

	CANPIDController lf_PID = lf.getPIDController();
    CANPIDController lb_PID = lb.getPIDController();
    CANPIDController rf_PID = rf.getPIDController();
	CANPIDController rb_PID = rb.getPIDController();
    
    Gyro gyro = new ADXRS450_Gyro(kOnboardCS0);

    PIDController aimPID = new PIDController(0.01, 0, 0);

    
    public Mecanum() 
    {
        md.setMaxOutput(.44); // spitfire

        // md.setMaxOutput(.35); minispit

        gyro.reset();
        gyro.calibrate();
        
        m_rightFrontEncoder.setPositionConversionFactor(0.0454);
        m_rightBackEncoder.setPositionConversionFactor(0.0454);
        m_leftFrontEncoder.setPositionConversionFactor(0.0454);
        m_leftBackEncoder.setPositionConversionFactor(0.0454);

        /* For spitfire, not for new robot
        lf.setInverted(true);
		lb.setInverted(true);
		rb.setInverted(true);
        */

        // Minispit
        rf.setInverted(true);

		lb.setIdleMode(kCoast);
		rf.setIdleMode(kCoast);
		rb.setIdleMode(kCoast);
		lf.setIdleMode(kCoast);

		lf.burnFlash();
		lb.burnFlash();
		rf.burnFlash();
        rb.burnFlash();
        
        distPID.setTolerance(0.01);
        turnPID.setTolerance(0.2);//0.2
        
        // turnPID.setIntegratorRange(0, 15);
    }
    
    
    // public void drive(double xSpeed, double ySpeed, double zRotation)
    // {
    //     this.drive(xSpeed, ySpeed, zRotation, false, false);
    // }

    // public void drive(double xSpeed, double ySpeed, double zRotation, boolean slowMode, boolean fieldOriented)
    // {
    //     double multiplier = slowMode ? 0.5 : 1;
        
    //     if (fieldOriented)
    //         md.driveCartesian(multiplier*xSpeed, multiplier*ySpeed, multiplier*zRotation, gyro.getAngle());
    //     else
    //         md.driveCartesian(multiplier*xSpeed, multiplier*ySpeed, multiplier*zRotation);
    // }
    
    public void drive(double xSpeed, double ySpeed, double zRotation)
    {
        md.driveCartesian(xSpeed, ySpeed, zRotation);
    }

    // displays the heading data in the gyro, form of a compass
	public void displayGyro()
	{
        // SmartDashboard.putNumber("gyro", gyro.getAngle());
        SmartDashboard.putData((Sendable) gyro);
    }

    public void displayEncoders()
    {
		SmartDashboard.putNumber("lf position", m_leftFrontEncoder.getPosition());
		SmartDashboard.putNumber("rf position", m_rightFrontEncoder.getPosition());
		SmartDashboard.putNumber("lb position", m_leftBackEncoder.getPosition());
        SmartDashboard.putNumber("rb position", m_rightBackEncoder.getPosition());
        
        SmartDashboard.putNumber("rb Velocity", m_rightBackEncoder.getVelocity());
        SmartDashboard.putNumber("elbeee Velocity", m_leftBackEncoder.getVelocity());
        SmartDashboard.putNumber("lf Velocity", m_leftFrontEncoder.getVelocity());
        SmartDashboard.putNumber("rf Velocity", m_rightFrontEncoder.getVelocity());
    }

    public void resetSensors()
    {
        gyro.reset();
        m_leftBackEncoder.setPosition(0);
        m_rightBackEncoder.setPosition(0);
        m_leftFrontEncoder.setPosition(0);
        m_rightFrontEncoder.setPosition(0);
    }
    
    
    // 1.0/15 is for carpet
    // 1.0/20 is for sully room (for now) HMMM
    /// update: never drive in sully's room, it's too slippery

    // PIDController turnPID = new PIDController(0.066667, 0, 0);

    // from frcpdr
    // PIDController turnPID = new PIDController(0.12, 0.49, 0.0073);
    // from wpiman
    // PIDController turnPID = new PIDController(0.12, 0.8223, 0.01216);
    // mean of two
    // PIDController turnPID = new PIDController(0.12, 0.65, 0.01);
    // softer??
    // PIDController turnPID = new PIDController(0.1, 0.6, 0.008);
    // SOFTER II
    // PIDController turnPID = new PIDController(0.025, 0.5, 0.0075);

    // from frcpdr; Ziegler-Nichols PI only loop. slow but works
    PIDController turnPID = new PIDController(0.09, 0.22, 0);

    public boolean turnToAngle(double angle)
    {
        md.driveCartesian(0, 0, turnPID.calculate(gyro.getAngle(), angle));

        return turnPID.atSetpoint();
    }

    PIDController distPID = new PIDController(2.5, 0, 0);

    public boolean driveToDistance(double distance)
    {
        double distanceOutput = distPID.calculate(m_leftFrontEncoder.getPosition(), distance);
        // double turnOutput = turnPID.calculate(gyro.getAngle(), 0);
        double turnOutput = 0;
        
        md.driveCartesian(0, distanceOutput, turnOutput);

        return distPID.atSetpoint();
    }
}
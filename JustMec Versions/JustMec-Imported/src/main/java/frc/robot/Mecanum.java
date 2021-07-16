package frc.robot;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.*;

import java.io.IOException;
import java.nio.file.Path;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;

public class Mecanum
{
    private static CANSparkMax lf = new CANSparkMax(1, kBrushless);
    private static CANSparkMax lb = new CANSparkMax(6, kBrushless);
    private static CANSparkMax rf = new CANSparkMax(2, kBrushless);
    private static CANSparkMax rb = new CANSparkMax(5, kBrushless);

    MecanumDrive md = new MecanumDrive(lf, lb, rf, rb);

    CANEncoder m_leftFrontEncoder = lf.getEncoder();
    CANEncoder m_leftBackEncoder = lb.getEncoder();
    CANEncoder m_rightFrontEncoder = rf.getEncoder();
    CANEncoder m_rightBackEncoder = rb.getEncoder();

	CANPIDController lf_PID = lf.getPIDController();
    CANPIDController lb_PID = lb.getPIDController();
    CANPIDController rf_PID = rf.getPIDController();
	CANPIDController rb_PID = rb.getPIDController();
    
    Translation2d m_frontLeftLocation = new Translation2d(0.3048, 0.2286);
    Translation2d m_frontRightLocation = new Translation2d(0.3048, -0.2286);
    Translation2d m_backLeftLocation = new Translation2d(-0.3048, -0.3048);
    Translation2d m_backRightLocation = new Translation2d(-0.3048, -0.3048);
    
    Gyro m_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

    MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(m_kinematics, new Rotation2d(-m_gyro.getAngle()), new Pose2d(5.0, 13.5, new Rotation2d()));    
    
    Trajectory trajectory;
    // TODO: Make a RamseteController here
    // (Note: In the 2021 WPILib update, HolonomicDriveController will be available in the full release)
    HolonomicDriveController controller = new HolonomicDriveController(
        new PIDController(1, 0, 0), new PIDController(1, 0, 0),
        new ProfiledPIDController(1, 0, 0,
          new TrapezoidProfile.Constraints(6.28, 3.14)));
      // Here, our rotation profile constraints were a max velocity
      // of 1 rotation per second and a max acceleration of 180 degrees
      // per second squared.
    
    // Creating my kinematics object using the wheel locations.
    MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

    PIDController aimPID = new PIDController(0.01, 0, 0);

    
    public Mecanum() 
    {
        md.setMaxOutput(.5);
        m_gyro.reset();
        m_gyro.calibrate();
        // m_rightFrontEncoder.setPositionConversionFactor(.1524*Math.PI);
        // m_rightBackEncoder.setPositionConversionFactor(.1524);
        // m_leftFrontEncoder.setPositionConversionFactor(.1524);
        // m_leftBackEncoder.setPositionConversionFactor(.1524); // .1524 is 6 inches in meters
        m_rightFrontEncoder.setPositionConversionFactor(0.0454);
        m_rightBackEncoder.setPositionConversionFactor(0.0454);
        m_leftFrontEncoder.setPositionConversionFactor(0.0454);
        m_leftBackEncoder.setPositionConversionFactor(0.0454);


        lf.setInverted(true);
		lb.setInverted(true);
		rb.setInverted(true);

		lf.setIdleMode(IdleMode.kCoast);
		lb.setIdleMode(IdleMode.kCoast);
		rf.setIdleMode(IdleMode.kCoast);
		rb.setIdleMode(IdleMode.kCoast);

		lf.burnFlash();
		lb.burnFlash();
		rf.burnFlash();
        rb.burnFlash();
        


    
        // double choice = SmartDashboard.getNumber("Auton choice", 1);
        // String trajectoryJSON = "paths/" + choice + ".wpilib.json";

        // try {
        //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        //     trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        // } catch (IOException ex) {
        //     DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        // }
    }
    
    /**
     * drives EX. "vroom vroom"
     */
    public void drive(double xSpeed, double ySpeed, double zRotation)
    {
        // TODO: add slowmode toggle to both drivetrains
        md.driveCartesian(xSpeed, ySpeed, zRotation);
    }
    
    public void followTrajectory(double time)
    {
        //  represents next position of the robot
        Trajectory.State goal = trajectory.sample(time);
        //  gives the speed for left and right sides        
        ChassisSpeeds adjustedSpeeds = controller.calculate(this.getLocation(), goal, goal.poseMeters.getRotation()); // should be the *current*
                                                                                             // robot pose
        
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);
        
        lf_PID.setReference(wheelSpeeds.frontLeftMetersPerSecond, ControlType.kVelocity);
        lb_PID.setReference(wheelSpeeds.rearLeftMetersPerSecond, ControlType.kVelocity);
        rf_PID.setReference(wheelSpeeds.frontRightMetersPerSecond, ControlType.kVelocity);
        rb_PID.setReference(wheelSpeeds.rearRightMetersPerSecond, ControlType.kVelocity);
    }


    public void updateOdometry()
    {
        m_odometry.update(new Rotation2d(-m_gyro.getAngle()), new MecanumDriveWheelSpeeds(
                m_leftFrontEncoder.getVelocity(),
                m_leftBackEncoder.getVelocity(),
                m_rightFrontEncoder.getVelocity(),
                m_rightBackEncoder.getVelocity()));

        
        SmartDashboard.putNumber("x", m_odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("y", m_odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("rot", m_odometry.getPoseMeters().getRotation().getDegrees());
    }

    public Pose2d getLocation()
    {
        return m_odometry.getPoseMeters();
    }

    public void resetPose()
    {
        m_gyro.reset();
        m_odometry.resetPosition(new Pose2d(), new Rotation2d(0));
    }

    // displays the heading data in the gyro, form of a compass
	public void displayGyro()
	{
        SmartDashboard.putNumber("gyro", m_gyro.getAngle());
    }

    public void displayEncoders()
    {        
		SmartDashboard.putNumber("lf speed", m_leftFrontEncoder.getPosition());
		SmartDashboard.putNumber("rf speed", m_rightFrontEncoder.getPosition());
		SmartDashboard.putNumber("lb speed", m_leftBackEncoder.getPosition());
		SmartDashboard.putNumber("rb speed", m_rightBackEncoder.getPosition());
    }

	public void resetSensors() {
        m_gyro.reset();
        m_leftBackEncoder.setPosition(0);
        m_rightBackEncoder.setPosition(0);
        m_leftFrontEncoder.setPosition(0);
        m_leftFrontEncoder.setPosition(0);
        
    }
    
    PIDController turnPID = new PIDController(1.0/15, 0, 0);

    public void turnToAngle(double angle)
    {
        md.driveCartesian(0, 0, turnPID.calculate(m_gyro.getAngle(), angle));
    }

    PIDController distPID = new PIDController(2.5, 0, 0);

    public void driveToDistance(double distance)
    {
        double error = m_leftFrontEncoder.getPosition() - m_rightFrontEncoder.getPosition() / 2;
        md.driveCartesian(0, distPID.calculate(error, distance), 0);
    }
}
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;

import com.ctre.phoenix.motorcontrol.ControlMode;
import static com.ctre.phoenix.motorcontrol.NeutralMode.*;

// Wow. Wrapper class make good !!! 😀
public class Shooter420
{
    CRRFalcon500 shootMotorLeft = new CRRFalcon500(Constants.LEFT_SHOOTER_ID, true, Coast, Constants.SHOOTER_PID_GAINS);
    CRRFalcon500 shootMotorRight = new CRRFalcon500(Constants.RIGHT_SHOOTER_ID, false, Coast, shootMotorLeft);   //  follows left motor
    // What if we stored motor instances in Constants... plz

    // TalonSRX loadMotor = new TalonSRX(Constants.LOAD_MOTOR_ID);
    
    // PIDController pid = new PIDController(0.0002, 0, 0);

    final double maxVelocity = 6380;
    final double maxAcceleration = 1000000000;

    ProfiledPIDController pid = new ProfiledPIDController(0.0002, 0, 0, new Constraints(maxVelocity, maxAcceleration));

    public void shoot()
    {
        // shootMotorLeft.setVelocity(1700);
        // shootMotorLeft.set(pid.calculate(shootMotorLeft.getVelocity(), 2000));
        shootMotorLeft.set(1);


        // if (shootMotorLeft.atSetpoint(50));
            // loadMotor.set(ControlMode.PercentOutput, 1);
    }

    public void stop()
    {
        // shootMotorLeft.setVelocity(0);
        shootMotorLeft.set(0);
        // shootMotorLeft.set(pid.calculate(shootMotorLeft.getVelocity(), 0));

        // loadMotor.set(ControlMode.PercentOutput, 0);
    }

    public void printSpeed()
    {
        SmartDashboard.putNumber("left falocon", shootMotorLeft.getVelocity());
        SmartDashboard.putNumber("right falocon", shootMotorRight.getVelocity());
    }
}
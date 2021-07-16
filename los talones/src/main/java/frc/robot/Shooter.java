package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

public class Shooter
{        
    TalonFX shootMotorLeft =  new TalonFX(Constants.LEFT_SHOOTER_ID);
    // TalonFX shootMotorRight = new TalonFX(Constants.RIGHT_SHOOTER_ID);

    TalonSRX loadMotor = new TalonSRX(Constants.LOAD_MOTOR_ID);
    
    public Shooter()
    {
        loadMotor.setInverted(true);    //  TODO: does it need inversion?
        // shootMotorRight.follow(shootMotorLeft);
        // shootMotorRight.setInverted(true);
        
        shootMotorLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        double kP = Constants.SHOOTER_PID_GAINS.kP;
        // use 0.5 * kP on talons, since one revolution is 2048 native units (see RPMtoTalonUnits() below)
        shootMotorLeft.config_kP(0, 0.5 * kP);
    }

    public void shoot()
    {
        shootMotorLeft.set(TalonFXControlMode.Velocity, RPMtoTalonUnits(6380));
        
        if (shootMotorLeft.getClosedLoopError(0) < RPMtoTalonUnits(50)) //  make sure to use the method EVERYWHERE you want to reference RPM!
            loadMotor.set(ControlMode.PercentOutput, 1);
    }

    public void stop()
    {
        shootMotorLeft.set(TalonFXControlMode.Velocity, RPMtoTalonUnits(0));
        loadMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Converts revolutions per minute into TalonFX native velocity units.
     * Falcon 500's max velocity is 6380 RPM.
     * @param rpm velocity in revolutions per minute
     * @return converted velocity that you can pass into set(TalonFXControlMode.Velocity, [converted velocity here])
     */
    private double RPMtoTalonUnits(double rpm)
    {        
        // So the TalonFX uses "units/100ms", not RPM in the Velocity control mode
        
        // there are 2048 units per revolution on a Falcon        
        double unitsPerMinute = rpm * 2048;
        
        // and 60000 ms per min
        // so, 1/60000 min per ms
        // so, 1/600 min per 100ms
        double unitsPer100ms = unitsPerMinute / 600;

        return unitsPer100ms;
    }
}
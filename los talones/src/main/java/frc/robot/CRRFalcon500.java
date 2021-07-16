
package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import frc.robot.Constants.PIDGains;

    // pro tip when reading this class: minimize methods you don't want to read
    // shortcut is ctrl+k, then ctrl+0 to fold everything
    // ctrl k [ folds
    // ctrl k ] unfolds

public class CRRFalcon500 extends WPI_TalonFX
{
    // YOU CAN NEVER HAVE ENOUGH CONSTRUCTOR OVERLOADS, MY FRIEND - Josh, 2021

    /**
     * Constructs a Falcon 500.
     * Note: Sets NeutralMode to Coast by default.
     * @param id CAN ID of the device... may have to configure it in Phoenix Tuner
     */
    public CRRFalcon500(int id) { this(id, false, NeutralMode.Coast); }

    /**
     * Constructs a Falcon 500.
     * @param id CAN ID of the device... may have to configure it in Phoenix Tuner
     * @param inverted Whether or not to invert the motor's direction of rotation.
     * @param mode What to do when input is 0 (Brake or Coast; brake will actively stop the motor and Coast will let it spin freely)
     */
    public CRRFalcon500(int id, boolean inverted, NeutralMode mode)
    {
        super(id);
        this.configFactoryDefault();
        this.setInverted(inverted);
        this.setNeutralMode(mode);
    }
    
    /**
     * Constructs a Falcon 500.
     * @param id CAN ID of the device... may have to configure it in Phoenix Tuner
     * @param inverted Whether or not to invert the motor's direction of rotation.
     * @param mode What to do when input is 0 (Brake or Coast; brake will actively stop the motor and Coast will let it spin freely)
     * @param leaderToFollow Master motor to copy inputs of. Useful for when you have multiple motors on the same mechanism. (You can pass in a CRRFalcon500 here.)
     */
    public CRRFalcon500(int id, boolean inverted, NeutralMode mode, WPI_TalonFX leaderToFollow)
    {
        this(id, inverted, mode);

        this.follow(leaderToFollow, FollowerType.PercentOutput);   //  would we ever use auxoutput??????
    }
    
    /**
     * Constructs a Falcon 500.
     * @param id CAN ID of the device... may have to configure it in Phoenix Tuner
     * @param inverted Whether or not to invert the motor's direction of rotation.
     * @param mode What to do when input is 0 (Brake or Coast; brake will actively stop the motor and Coast will let it spin freely)
     * @param gains PID constants. 
     * <p> Usage: CRRFalcon500 shooterMotor = new CRRFalcon500(0, false, NeutralMode.Coast, Constants.ShooterPIDGains);
     * <br> ///Constants.java: PIDGains ShooterPIDGains = new PIDGains(0.1, 0, 0);
     */
    public CRRFalcon500(int id, boolean inverted, NeutralMode mode, PIDGains gains)
    {
        this(id, inverted, mode);

        this.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        
        setPIDGains(gains);
    }
    
    /**
     * Sets the PIDGains for closed-loop control modes. Tip: use the constructor unless you want to change this periodically for tuning
     * @param gains
     */
    public void setPIDGains(PIDGains gains)
    {
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        var pidconfig = talonConfig.slot0;

        pidconfig.kP = gains.kP * 0.5;
        pidconfig.kI = gains.kI * 0.0005;
        pidconfig.kD = gains.kD * 500;

        this.configAllSettings(talonConfig);
    }

    /**
     * Uses the closed-loop velocity control of the TalonFX. <br>
     * MUST set PID constants with the constructor or setPIDGains().
     * @param rpm Angular velocity you want to set the motor to in revolutions per minute. <br>
     * The Falcon 500's max free speed is 6380 RPM.
     */
    public void setVelocity(double rpm) { this.set(TalonFXControlMode.Velocity, RPMtoTalonUnits(rpm)); }

    /**
     * Gets the current RPM of the Falcon, can be used whether you're using closed-loop velocity control or not (probably?)
     * @return current angular velocity in RPM
     */
    public double getVelocity() { return TalonUnitstoRPM(this.getSelectedSensorVelocity()); }

    double rpmErrorAllowed;
    /**
     * Checks if the closed loop velocity control is near the setpoint.
     * @param rpmErrorAllowed how much error is acceptable in revolutions per minute
     * @return true if error < rpmErrorAllowed
     */
    public boolean atSetpoint(double rpmErrorAllowed) { setRpmErrorAllowed(rpmErrorAllowed); return this.getClosedLoopError() < RPMtoTalonUnits(this.rpmErrorAllowed); }

    /**
     * Sets the allowed velocity error in velocity control mode
     * @param rpmErrorAllowed
     */
    public void setRpmErrorAllowed(double rpmErrorAllowed) { this.rpmErrorAllowed = rpmErrorAllowed; }

    /**
     * Sets normal power, useful when you don't need closed-loop velocity control.
     */
    public void set(double power) { this.set(TalonFXControlMode.PercentOutput, power);  }

    /**
     * Uses the Talon's built-in MusicTone control mode to play a tone.
     * You can't set the motor while it sings though.
     * @param frequency Frequency of the tone to play in Hertz
     */
    public void sing(double frequency) { this.set(TalonFXControlMode.MusicTone, frequency); }

    /**
     * Converts revolutions per minute into TalonFX native velocity units.
     * Falcon 500's max velocity is 6380 RPM.
     * @param rpm velocity in revolutions per minute
     * @return converted velocity that you can pass into set(TalonFXControlMode.Velocity, [converted velocity here])
     */
    private double RPMtoTalonUnits(double rpm)
    {        
        // So the TalonFX uses "encoder units/100ms", not RPM in the Velocity control mode
        
        // there are 2048 units per revolution on a Falcon
        double unitsPerMinute = rpm * 2048;
        
        // and 60000 ms per min
        // so, 1/60000 min per ms
        // so, 1/600 min per 100ms
        double unitsPer100ms = unitsPerMinute / 600;

        return unitsPer100ms;
    }

    private double TalonUnitstoRPM(double value)
    {
        double revolutionsPer100ms = value / 2048;
        double RPM = revolutionsPer100ms * 600;
        
        return RPM;
    }
}

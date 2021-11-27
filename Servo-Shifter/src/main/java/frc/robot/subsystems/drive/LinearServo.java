package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * Sample code provided by FRC 4414.
 * The provided code is to control the position of the L16-R Miniature Linear Servo Actuators.
 * 
 * Modified to implement Subsystem to handle periodic functions.
 */
public class LinearServo extends Servo implements Subsystem
{
    double m_speed;
    double m_length;
    double setPos;
    double curPos;

    /**
     * Parameters for L16-R Actuonix Linear Actuators
     *
     * @param channel PWM channel used to control the servo
     * @param length  max length of the servo [mm]
     * @param speed   max speed of the servo [mm/second]
     */
    public LinearServo(int channel, int length, int speed)
    {
        super(channel);
        setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        m_length = length;
        m_speed = speed;
        
        register(); //  registers this subsystem with the Command Scheduler, to run the periodic() method
    }

    /**
     * Run this method in any periodic function to update the position estimation of
     * your servo
     *
     * @param setpoint the target position of the servo [mm]
     */
    public void setPosition(double setpoint)
    {
        setPos = MathUtil.clamp(setpoint, 0, m_length);
        setSpeed((setPos / m_length * 2) - 1);
    }

    double lastTime = 0;

    /**
     * Run this method in any periodic function to update the position estimation of
     * your servo
     */
    public void updateCurPos()
    {
        double dt = Timer.getFPGATimestamp() - lastTime;
        if (curPos > setPos + m_speed * dt) {
            curPos -= m_speed * dt;
        } else if (curPos < setPos - m_speed * dt) {
            curPos += m_speed * dt;
        } else {
            curPos = setPos;
        }
    }

    /**
     * Current position of the servo, must be calling {@link #updateCurPos()
     * updateCurPos()} periodically
     *
     * @return Servo Position [mm]
     */
    public double getPosition()
    {
        return curPos;
    }

    /**
     * Checks if the servo is at its target position, must be calling
     * {@link #updateCurPos() updateCurPos()} periodically
     * 
     * @return true when servo is at its target
     */
    public boolean isFinished()
    {
        return curPos == setPos;
    }

    @Override
    public void periodic()
    {
        updateCurPos();
    }
}
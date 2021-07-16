package frc.robot;

public class Constants
{
     //  TODO: tune
    public static final PIDGains SHOOTER_PID_GAINS = new PIDGains(0.01, 0, 0);
    
    public static final int LEFT_SHOOTER_ID = 0;
    public static final int RIGHT_SHOOTER_ID = 1;
    public static final int LOAD_MOTOR_ID = 40000;   //  TODO: wat id ?? add to can 2 plz

    /**
     * Stores PID gain constants.
     * <p> Usage: new PIDGains(kP, kI, kD) passed as an argument, with kP as your P constant and so on.
     */
    public static class PIDGains
    {
        double kP, kI, kD;
        
        public PIDGains(double kP, double kI, double kD)
        {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }
}

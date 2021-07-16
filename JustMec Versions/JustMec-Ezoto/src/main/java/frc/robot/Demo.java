package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class Demo
{
    Timer timer = new Timer();
    Mecanum dt = new Mecanum();

    /// Incrementally more complex autons:
    protected void Demos()
    {
        double time = timer.get();
        

        /// 1. No feedback whatsoever
        dt.drive(0, 1, 0);

        /// 2. Limit by time
        if (0 <= time && time <= 5)
            dt.drive(0, 1, 0);

        /// 3. Limit by sensors --> real world feedback         
        if (true/*encoder.getPosition() < 1 meter*/)
            dt.drive(0, 0.5, 0);
        
        if (true /*gyro.getAngle() < 90*/)
            dt.drive(0, 0, 0.5);
        
        /// 3.5: PID with sensors --> accurate and fast movement
            //  we've written methods in the drivetrain class already!
        dt.driveToDistance(1); // encoders
        dt.turnToAngle(90); // gyroscope

        /// 4. Limit by both time and sensors to execute actions one after another
        if (time < 5)
            dt.driveToDistance(1);
        else if (time < 10)
            dt.turnToAngle(90);


        // Below, we reset the sensors after every action
        if (time < 5)
        	dt.driveToDistance(1);
        else if (time < 5.1)
        	dt.resetSensors();
        else if (time < 10)
        	dt.driveToDistance(1);
        else if (time < 10.1)
            dt.resetSensors();

        /// Now... this looks like something you could put in a method...
        // Every time we want to do an action, we have to reset the sensors afterwards...
        // Go to Robot.java to see the implementation
    }
}

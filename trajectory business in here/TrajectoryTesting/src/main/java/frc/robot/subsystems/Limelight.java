package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    // PUBLIC METHODS:
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);  
    boolean v = tv.getBoolean(false);

    //      getTv()
    public boolean getTv(){
        return v;
    }

    //     getTx()
    public double getTx(){
        return x;
    }

    //      getTy()
    public double getTy() {
        return y;
    }

    //     getTa()
    public double getTa(){
        return area;
    }
    
    public void Update() {
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        v = tv.getBoolean(false);    
    }

    public void shuffleboardUpdate() {
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putBoolean("LimelightV",v);
        
    }
     
    public void toggleLimelightLED() {
        if(table.getEntry("ledMode").getDouble(1) == 1 ) //Light is off
        {
            table.getEntry("ledMode").setNumber(3); //Turn it on
        }
        else
        {
            table.getEntry("ledMode").setNumber(1); //Turn it off
        }
    }
    
    public void toggleLimelightBlinkLED(){
        if(table.getEntry("ledMode").getDouble(2) == 2 ) //Light is off
        {
            table.getEntry("ledMode").setNumber(1); //Turn it off
        }
        else
        {
            table.getEntry("ledMode").setNumber(2); //Turn it on
        }
    }
}
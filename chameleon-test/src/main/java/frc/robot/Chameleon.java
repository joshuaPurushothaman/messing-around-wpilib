package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.IVisionSystem;

public class Chameleon implements IVisionSystem
{
    NetworkTable table;
    NetworkTableEntry targetX, targetY, targetArea, targetValid;
    static final String CAMERA_NAME = "MyCamName";  //  TODO: nae nae?

    public Chameleon()
    {
        table=NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable(CAMERA_NAME);
        targetX=table.getEntry("yaw");
        targetY=table.getEntry("pitch");
        targetArea=table.getEntry("area");  //  TODO: unsure about area and valid keys
        targetValid=table.getEntry("isValid");
    }

    /**
     * removes the overlay in camera feed so driver can see more clearly
     * @param mode if true, overlay will not be shown
     */
    public void setDriverMode(boolean mode)
    {
        table.getEntry("driver_mode").setBoolean(mode);
    }

    @Override
    public double getX()
    {
        return targetX.getDouble(0);
    }

    @Override
    public double getY()
    {
        return targetY.getDouble(0);
    }

    @Override
    public double getArea()
    {
        return targetArea.getDouble(0);
    }

    @Override
    public boolean getValidTarget()
    {
        return targetValid.getBoolean(false);
    }
}

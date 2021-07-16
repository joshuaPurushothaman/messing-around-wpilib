package frc.robot.subsystems.drive;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.Solenoid;

public class ShiftingDrive extends Drivetrain
{
    CANEncoder lfEncoder = lf.getEncoder();
    CANEncoder rfEncoder = rf.getEncoder();

    Solenoid shifter = new Solenoid(0, 1);
    
    // TODO: set velocity conversion factors

    public void setHighGear(boolean isHighGear)
    {
        shifter.set(isHighGear);
    }

    @Override
    public void periodic()
    {
        super.periodic();

        updateShiftStatus();
    }

    private void updateShiftStatus()
    {
        final double shiftVel = 0;
        
        setHighGear(Math.abs(lfEncoder.getVelocity() + rfEncoder.getVelocity()) > shiftVel);    
    }    
}

package frc.robot.subsystems;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.system.plant.*;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;

public class DrivetrainSim extends TrajDrivetrain
{
    ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(gyro);

    SimDeviceSim lfSim = new SimDeviceSim("SPARK MAX [" + Constants.LF_MOTOR_ID + "]");
    SimDeviceSim rfSim = new SimDeviceSim("SPARK MAX [" + Constants.RF_MOTOR_ID + "]");
    SimDeviceSim lbSim = new SimDeviceSim("SPARK MAX [" + Constants.LB_MOTOR_ID + "]");
    SimDeviceSim rbSim = new SimDeviceSim("SPARK MAX [" + Constants.RB_MOTOR_ID + "]");

    SimDouble lfOutputSim = lfSim.getDouble("Applied Output");
    SimDouble rfOutputSim = rfSim.getDouble("Applied Output");
    SimDouble lbOutputSim = lbSim.getDouble("Applied Output");
    SimDouble rbOutputSim = rbSim.getDouble("Applied Output");
    
    SimDouble lfPositionSim = lfSim.getDouble("Position");
    SimDouble rfPositionSim = rfSim.getDouble("Position");
    SimDouble lbPositionSim = lbSim.getDouble("Position");
	SimDouble rbPositionSim = rbSim.getDouble("Position");
	
    SimDouble lfVelocitySim = lfSim.getDouble("Velocity");
    SimDouble rfVelocitySim = rfSim.getDouble("Velocity");
    SimDouble lbVelocitySim = lbSim.getDouble("Velocity");
    SimDouble rbVelocitySim = rbSim.getDouble("Velocity");

    // example set distance
    // positionProp.set(100.0);
    // realSpark.getAltEncoder().getPosition(); // 100.0
    
	// Create our feedforward gain constants (from the characterization
	// tool)
	static final double KvLinear = 1.98;
	static final double KaLinear = 0.2;
	static final double KvAngular = 1.5;
	static final double KaAngular = 0.3;


	// Create the simulation model of our drivetrain.
	private DifferentialDrivetrainSim dtSim = new DifferentialDrivetrainSim(
		// Create a linear system from our characterization gains.
		LinearSystemId.identifyDrivetrainSystem(KvLinear, KaLinear, KvAngular, KaAngular),
		DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
		10.71,                    // 10.71:1 gearing reduction.
		0.69,                  // The track width is 0.69 meters.
		Units.inchesToMeters(6), // The robot uses 6" radius wheels.
		null
	);

    Field2d field = new Field2d();
    
    public DrivetrainSim()
    {
        super();
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic()
    {
        super.periodic();

		field.setRobotPose(odometry.getPoseMeters());
    }
    
    @Override
    public void simulationPeriodic()
    {
        
		// Set the inputs to the system. Note that we need to convert
		// the [-1, 1] PWM signal to voltage by multiplying it by the
		// robot controller voltage.
		dtSim.setInputs(lfOutputSim.get() * RobotController.getInputVoltage(),
            rfOutputSim.get() * RobotController.getInputVoltage());

        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        dtSim.update(0.02);

        // Update all of our sensors.
        lfPositionSim.set(dtSim.getLeftPositionMeters());
        lfVelocitySim.set(dtSim.getLeftVelocityMetersPerSecond());
        rfPositionSim.set(dtSim.getRightPositionMeters());
        rfVelocitySim.set(dtSim.getRightVelocityMetersPerSecond());
        gyroSim.setAngle(-dtSim.getHeading().getDegrees());
    }
}

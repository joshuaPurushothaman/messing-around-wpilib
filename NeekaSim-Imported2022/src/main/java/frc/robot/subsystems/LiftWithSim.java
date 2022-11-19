package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class LiftWithSim extends Lift
{
	private final ElevatorSim elevatorSim = new ElevatorSim(
		DCMotor.getNEO(2),
		20/1,
		4.0,
		Units.inchesToMeters(1.76),
		LOW_POSITION_METERS,
		HIGH_POSITION_METERS,
		null
		// VecBuilder.fill(0.01)
	);

	// EncoderSim encoderSim = new EncoderSim(liftEncoder);

	  // Create a Mechanism2d visualization of the elevator
	private final Mechanism2d mech2d = new Mechanism2d(20, 50);
	private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 10, 0);
	private final MechanismLigament2d elevatorMech2d =
		mech2dRoot.append(
			new MechanismLigament2d(
				"Elevator", Units.metersToInches(elevatorSim.getPositionMeters()), 90));

	public LiftWithSim()
	{
		super();
		
		SmartDashboard.putData("Elevator sim", mech2d);
	}

	@Override
	public void simulationPeriodic()
	{
		// In this method, we update our simulation of what our elevator is doing
		// First, we set our "inputs" (voltages)
		elevatorSim.setInput(liftMotor.get() * RobotController.getBatteryVoltage());

		// Next, we update it. The standard loop time is 20ms.
		elevatorSim.update(0.020);

		// Finally, we set our simulated encoder's readings and simulated battery voltage
		// encoderSim.setDistance(elevatorSim.getPositionMeters());
		liftEncoder.setPosition(elevatorSim.getPositionMeters());

		// SimBattery estimates loaded battery voltage
		RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

		// Update the mechanism visualization
		elevatorMech2d.setLength(Units.metersToInches(elevatorSim.getPositionMeters()));

		// SimBattery estimates loaded battery voltages
		RoboRioSim.setVInVoltage(
			BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
	}
}

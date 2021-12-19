package frc.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants;

/**
 * GreyT Elevator v2
 */
public class Lift extends SubsystemBase
{
	PWMSparkMax liftMotor = new PWMSparkMax(Constants.LIFT_MOTOR_ID);

	Encoder liftEncoder = new Encoder(Constants.LIFT_ENCODER_A, Constants.LIFT_ENCODER_B);

	private static final double kElevatorDrumRadius = Units.inchesToMeters(1.76) / 2;
	private static final double kCountsPerRevolution = 360;
	
	public static final double HIGH_POSITION_METERS = Units.feetToMeters(6);
	public static final double LOW_POSITION_METERS = Units.feetToMeters(0);

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

	EncoderSim encoderSim = new EncoderSim(liftEncoder);

	public Lift()
	{
		liftEncoder.setDistancePerPulse(2.0 * Math.PI * kElevatorDrumRadius / kCountsPerRevolution);
	}

	public void set(double power)
	{
		liftMotor.set(power);
	}
	
	public double getPositionMeters()
	{
		return liftEncoder.getDistance();
	}

	@Override
	public void periodic()
	{
		SmartDashboard.putNumber("Lift Height", getPositionMeters());
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
		encoderSim.setDistance(elevatorSim.getPositionMeters());
		// SimBattery estimates loaded battery voltages
		RoboRioSim.setVInVoltage(
			BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
	}
}
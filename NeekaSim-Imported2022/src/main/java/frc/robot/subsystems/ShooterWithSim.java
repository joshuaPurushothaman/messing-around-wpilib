package frc.robot.subsystems;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.*;

//zoheb is king of the code
public class ShooterWithSim extends Shooter
{
	FlywheelSim flywheelSim = new FlywheelSim(
		DCMotor.getNEO(2), 
		1,
		// 0.5 * Units.lbsToKilograms(0.53) * Math.pow(1.5, 2));	//	moment of inertia of 4x1.5 colson wheel
		0.5);

	EncoderSim encoderSim = new EncoderSim(encoder);

	double position = 0;

	public ShooterWithSim()
	{
		super();

		REVPhysicsSim.getInstance().addSparkMax(leftMotor, DCMotor.getNEO(1));
		REVPhysicsSim.getInstance().addSparkMax(rightMotor, DCMotor.getNEO(1));
	}

	@Override
	public void simulationPeriodic()
	{
		flywheelSim.setInputVoltage(leftMotor.get() * RobotController.getInputVoltage());
		flywheelSim.update(0.02);
		// encoderSim.setRate(flywheelSim.getAngularVelocityRPM());
		position += flywheelSim.getAngularVelocityRPM() * 0.02 / 60;
		encoderSim.setRate(flywheelSim.getAngularVelocityRPM());

		RoboRioSim.setVInVoltage(
			BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
	}
}

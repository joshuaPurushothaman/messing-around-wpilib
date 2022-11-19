package frc.robot.subsystems;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.math.system.plant.DCMotor;

import com.revrobotics.REVPhysicsSim;

// import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;

public class DrivetrainWithSim extends Drivetrain
{
	DifferentialDrivetrainSim drivetrainSim = new DifferentialDrivetrainSim(
		DCMotor.getNEO(2),
		10.71,
		// DCMotor.getNEO(3),
		// 7.56,
		0.48866695903563, // from CAD
		7.9043, // from CAD
		// 54, // from CAD
		kWheelDiameterMeters / 2,
		kTrackWidthMeters,
		// The standard deviations for measurement noise:
		// x and y:          0.001 m
		// heading:          0.001 rad
		// l and r velocity: 0.1   m/s
		// l and r position: 0.005 m
		// VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
		null
	);

	ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(gyro);

	public DrivetrainWithSim()
	{
		super();

		REVPhysicsSim.getInstance().addSparkMax(leftMotor, DCMotor.getNEO(1));
		REVPhysicsSim.getInstance().addSparkMax(rightMotor, DCMotor.getNEO(1));
	}
	
	@Override
	public void resetPose(Pose2d pose)
	{
		super.resetPose(pose);

		if (RobotBase.isSimulation())
			drivetrainSim.setPose(pose);
	}

	@Override
	public void simulationPeriodic()
	{
		// Set the inputs to the system. Note that we need to convert
		// the [-1, 1] PWM signal to voltage by multiplying it by the
		// robot controller voltage.
		drivetrainSim.setInputs(leftMotor.get() * RobotController.getInputVoltage(),
							-rightMotor.get() * RobotController.getInputVoltage());

		// Advance the model by 20 ms. Note that if you are running this
		// subsystem in a separate thread or have changed the nominal timestep
		// of TimedRobot, this value needs to match it.
		drivetrainSim.update(0.02);

		// Update all of our sensors.
		// leftEncoderSim.setDistance(drivetrainSim.getLeftPositionMeters());
		// leftEncoderSim.setRate(drivetrainSim.getLeftVelocityMetersPerSecond());
		// rightEncoderSim.setDistance(drivetrainSim.getRightPositionMeters());
		// rightEncoderSim.setRate(drivetrainSim.getRightVelocityMetersPerSecond());
		leftEncoder.setPosition(drivetrainSim.getLeftPositionMeters());
		rightEncoder.setPosition(drivetrainSim.getRightPositionMeters());

		gyroSim.setAngle(-drivetrainSim.getHeading().getDegrees());
	}
}

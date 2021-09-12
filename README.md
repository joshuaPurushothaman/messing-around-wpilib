# messing-around-wpilib
This is a full upload of my FRC WPILib Java code from over the years. Some of it is old junk, but some of it can be really useful.

(The following descriptions may be outdated.)

DiffDriveSimTest - Command-Based. Tried to use the new Simulation features with a differential drivetrain with SPARK MAXes. It did not work.

Duck - Command-Based. Code for Duck; simple differential drive with TalonSRXs.

JustMec Versions:
  JustMec-Imported - TimedRobot. just mecanum drive code.
  JustMec-Ezoto - TimedRobot. Mecanum Drive code. Contains autonomous code, contains a copy of Skillz.java, which is what we used prior to switching to command based (If you are looking for a cool autonomous solution without using Command Based, this is where you want to look!)
  JustMec - do not use, is a 2020 version of justmec-imported
  Any other JustMec version files seem corrupted :(

SPARK-MAX-Examples-master - a copy of an old version of REV's SPARK MAX code examples. Find an updated version here: https://github.com/REVrobotics/SPARK-MAX-Examples

ShifterBot - Command based, attempt at JUST CODE for a pnuematic auto-shifting drivetrain. Probably doesn't really work, but does contain pnuematics code which might be useful to you...

SolenoidExample - TimedRobot, an old cloned copy of WPILib's SolenoidExample for pnuematics. Find latest versions here: https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/wpilib-examples.html

SwerveMaybee - Command based attempt at Swerve Drive code using newer WPILib features, and CANSparkMax.

chameleon-test - TimedRobot. Chameleon is a vision solution for FRC. https://chameleon-vision.readthedocs.io/en/latest/contents.html

los talones - TimedRobot. TalonFX (Falcon 500) testing code. Contains a potential wrapper class for the TalonFX API in CRRFalcon500.java. Check it out if you plan on using Falcons!

PurePursuitTest - Command Based, for Romi Robot. Attempt at Pure Pursuit trajectory following...

romi1 - Command based, for Romi Robot https://docs.wpilib.org/en/stable/docs/romi-robot/index.html Y'all should get one!

shootertesting - TimedRobot. Code for Spitfire (2020-2021 Infinite Recharge robot) shooter subsystem testing.

TalonFXDiffDrive - CommandBased, differential drive code for Falcon500/TalonFXs! Check it out if you plan on using a Falcon based drivetrain!

talontest - TimedRobot, some more old TalonFX testing code.

trajectory business in here:
  TrajectoryTesting-Imported - TimedRobot, some of my early days' trajectory testing code. Since it's trajectories in TimedRobot, I'd avoid it. Look for command based examples instead.
  TrajectoryTesting - do not use, is a 2020 version of TrajectoryTesting-Imported
  WPIEXAMPLE-DifferentialDriveBot and WPIEXAMPLE-MecanumDriveOdometry are both WPILib examples. https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/wpilib-examples.html
  cmdtrajinherit - Command based, an attempt at trajectory tracking! Check it out.

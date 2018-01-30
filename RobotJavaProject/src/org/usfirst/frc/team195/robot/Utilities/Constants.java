package org.usfirst.frc.team195.robot.Utilities;

import org.usfirst.frc.team195.robot.Utilities.SplineMotion.SRX.SRXTrajectoryConfig;

public class Constants {
	public static final boolean TUNING_PIDS = true;
	public static final boolean DEBUG = false;
	public static final boolean REPORTING_ENABLED = true;
	public static final boolean REPORT_TO_DRIVERSTATION_INSTEAD_OF_CONSOLE = false;
	public static final SRXTrajectoryConfig SXC = new SRXTrajectoryConfig();
	static {
		SXC.name = "StandardConfig";
		SXC.dt = .01;   //Time step in seconds
		SXC.max_acc = 30;   //Inches per sec^2
		SXC.max_jerk = 720.0;   //Inches per sec^3
		SXC.max_vel = 54.0; //Inches per sec
		SXC.wheelbaseWidthInches = 24.5;    //Inches
		SXC.wheelDiameterInches = 4.88; //Inches
		SXC.encoderRotToWheelRotFactor = 1; //Conversion from encoder rotations to wheel rotations
		SXC.encoderTicksPerRev = 4096;  //Encoder ticks per revolution of encoder
	}

	//Drive Config Attack 3
//	public static final int DRIVE_X_AXIS = 0;
//	public static final int DRIVE_Y_AXIS = 1;
//	public static final int DRIVE_IMM_TURN = 8;
//	public static final int DRIVE_SHIFT_LOW = 7;
//	public static final int DRIVE_SHIFT_HIGH = 6;
	
	//Drive Config F310
	public static final int DRIVE_X_AXIS = 2;
	public static final int DRIVE_Y_AXIS = 1;
	public static final int DRIVE_IMM_TURN = 7;
	public static final int DRIVE_HOLD_BRAKE = 8;
	public static final int DRIVE_SHIFT_LOW = 5;
	public static final int DRIVE_SHIFT_HIGH = 6;
	public static final int INTAKE_CLOSE_RUN = 5;
	public static final int INTAKE_RUN = 8;
	public static final int INTAKE_CLOSE = 1;
	public static final int INTAKE_OPEN = 2;
	public static final int INTAKE_RUN_REVERSE = 4;
	public static final int INTAKE_CLOSE_HALF = 3;
	
	public static final int kTimeoutMs = 20;
	public static final double kMotorDeadband = 0.01;
	public static final double kSensorUnitsPerRotation = 4096;




















	// Target parameters
	// Source of current values: https://firstfrc.blob.core.windows.net/frc2017/Manual/2017FRCGameSeasonManual.pdf
	// Section 3.13
	// ...and https://firstfrc.blob.core.windows.net/frc2017/Drawings/2017FieldComponents.pdf
	// Parts GE-17203-FLAT and GE-17371 (sheet 7)
	public static double kBoilerTargetTopHeight = 88.0;
	public static double kBoilerRadius = 7.5;

	// Shooter tuning parameters
	public static boolean kIsShooterTuning = false;
	public static double kShooterTuningRpmFloor = 2900;
	public static double kShooterTuningRpmCeiling = 3500;
	public static double kShooterTuningRpmStep = 50;
	public static double kShooterTuningRpm = 3500.0;

	/* ROBOT PHYSICAL CONSTANTS */

	// Wheels
	public static double kDriveWheelDiameterInches = 4.88;
	public static double kTrackWidthInches = 24.5;
	public static double kTrackScrubFactor = 1; // 0.924 ?

	// Geometry
	public static double kCenterToFrontBumperDistance = 15;
	public static double kCenterToIntakeDistance = 23.11;
	public static double kCenterToRearBumperDistance = 15;
	public static double kCenterToSideBumperDistance = 15;

	// Shooting suggestions
	public static double kOnTargetErrorThreshold = 3.0;

	// Intake Voltages
	public static double kIntakeVoltageMax = 7.5;
	public static double kIntakeVoltageMin = 5.5;
	public static double kIntakeShootingVoltage = 4.0;
	public static final double kIntakeVoltageDifference = kIntakeVoltageMax - kIntakeVoltageMin;

	/* CONTROL LOOP GAINS */

	// PID gains for drive velocity loop (HIGH GEAR)
	// Units: setpoint, error, and output are in inches per second.
	public static double kDriveHighGearVelocityKp = 1.2;
	public static double kDriveHighGearVelocityKi = 0.0;
	public static double kDriveHighGearVelocityKd = 6.0;
	public static double kDriveHighGearVelocityKf = .15;
	public static int kDriveHighGearVelocityIZone = 0;
	public static double kDriveHighGearVelocityRampRate = 240.0;
	public static double kDriveHighGearNominalOutput = 0.5;
	public static double kDriveHighGearMaxSetpoint = 17.0 * 12.0; // 17 fps

	// PID gains for drive velocity loop (LOW GEAR)
	// Units: setpoint, error, and output are in inches per second.
	public static double kDriveLowGearPositionKp = 1.0;
	public static double kDriveLowGearPositionKi = 0.002;
	public static double kDriveLowGearPositionKd = 100.0;
	public static double kDriveLowGearPositionKf = .45;
	public static int kDriveLowGearPositionIZone = 700;
	public static double kDriveLowGearPositionRampRate = 240.0; // V/s
	public static double kDriveLowGearNominalOutput = 0.5; // V
	public static double kDriveLowGearMaxVelocity = 6.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 6 fps
	// in RPM
	public static double kDriveLowGearMaxAccel = 18.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 18 fps/s
	// in RPM/s

	public static double kDriveVoltageCompensationRampRate = 0.0;

	// Turn to heading gains
	public static double kDriveTurnKp = 3.0;
	public static double kDriveTurnKi = 1.5;
	public static double kDriveTurnKv = 0.0;
	public static double kDriveTurnKffv = 1.0;
	public static double kDriveTurnKffa = 0.0;
	public static double kDriveTurnMaxVel = 360.0;
	public static double kDriveTurnMaxAcc = 720.0;

	// Shooter gains
	public static double kShooterTalonKP = 0.16;
	public static double kShooterTalonKI = 0.00008;
	public static double kShooterTalonKD = 0.0;
	public static double kShooterTalonKF = 0.035;
	public static double kShooterRampRate = 60.0;

	public static double kShooterTalonHoldKP = 0.0;
	public static double kShooterTalonHoldKI = 0.0;
	public static double kShooterTalonHoldKD = 0.0;

	public static double kShooterHoldRampRate = 720.0;

	public static int kShooterTalonIZone = 1000;// 73 rpm
	public static int kShooterOpenLoopCurrentLimit = 35;

	public static double kShooterSetpointDeadbandRpm = 1.0;

	// Used to determine when to switch to hold profile.
	public static double kShooterMinTrackStability = 0.25;
	public static double kShooterStartOnTargetRpm = 50.0;
	public static double kShooterStopOnTargetRpm = 150.0;
	public static int kShooterKfBufferSize = 20;
	public static int kShooterMinOnTargetSamples = 20; // Should be <= kShooterKvBufferSize

	public static int kShooterJamBufferSize = 30;
	public static double kShooterDisturbanceThreshold = 25;
	public static double kShooterJamTimeout = 1.5; // In secs
	public static double kShooterUnjamDuration = 0.5; // In secs
	public static double kShooterMinShootingTime = 1.0; // In secs

	public static double kShooterSpinDownTime = 0.25;

	// Feeder gains
	public static double kFeederKP = 0.02;
	public static double kFeederKI = 0.0;
	public static double kFeederKD = 0.2;
	public static double kFeederKF = 0.009;
	public static double kFeederRampRate = 240.0;
	public static double kFeederVoltageCompensationRampRate = 10.0;
	public static double kFeederFeedSpeedRpm = 5400.0;
	public static double kFeederSensorGearReduction = 3.0;

	// Hopper gains
	public static double kHopperRampRate = 48.0;

	// Do not change anything after this line unless you rewire the robot and
	// update the spreadsheet!
	// Port assignments should match up with the spreadsheet here:
	// https://docs.google.com/spreadsheets/d/12_Mrd6xKmxCjKtsWNpWZDqT7ukrB9-1KKFCuRrO4aPM/edit#gid=0

	/* TALONS */
	// (Note that if multiple talons are dedicated to a mechanism, any sensors
	// are attached to the master)

	// Drive
	public static final int kLeftDriveMasterId = 1;
	public static final int kLeftDriveSlaveId = 2;
	public static final int kLeftDriveSlaveId2 = 3;
	public static final int kRightDriveMasterId = 4;
	public static final int kRightDriverSlaveId = 5;
	public static final int kRightDriverSlaveId2 = 6;

	// Feeder
	public static final int kFeederMasterId = 8;
	public static final int kFeederSlaveId = 7;

	// Intake
	public static final int kIntakeMasterId = 5;
	public static final int kIntakeSlaveId = 10;

	// Hopper / Floor
	public static final int kHopperMasterId = 6;
	public static final int kHopperSlaveId = 9;

	// Shooter
	public static final int kRightShooterMasterId = 2;
	public static final int kRightShooterSlaveId = 1;
	public static final int kLeftShooterSlave1Id = 13;
	public static final int kLeftShooterSlave2Id = 14;

	// Gear Grabber
	public static final int kGearGrabberId = 15;

	// Solenoids
	public static final int kShifterSolenoidId1 = 0; // PCM 0, Solenoid 0
	public static final int kShifterSolenoidId2 = 1; // PCM 0, Solenoid 0
	public static final int kIntakeDeploySolenoidId = 1; // PCM 0, Solenoid 1
	public static final int kHopperSolenoidId = 2; // PCM 0, Solenoid 2
	public static final int kGearWristSolenoid = 7; // PCM 0, Solenoid 7

	// Analog Inputs
	public static int kLEDOnId = 2;

	// Digital Outputs
	public static int kGreenLEDId = 9;
	public static int kRangeLEDId = 8;

	// Phone
	public static int kAndroidAppTcpPort = 8254;

	// Path following constants
	public static double kMinLookAhead = 12.0; // inches
	public static double kMinLookAheadSpeed = 9.0; // inches per second
	public static double kMaxLookAhead = 24.0; // inches
	public static double kMaxLookAheadSpeed = 120.0; // inches per second
	public static double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
	public static double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

	public static double kInertiaSteeringGain = 0.02; // angular velocity command is multiplied by this gain *
	// our speed
	// in inches per sec
	public static double kSegmentCompletionTolerance = 2; // inches
	public static double kPathFollowingMaxAccel = 100.0; // inches per second^2
	public static double kPathFollowingMaxVel = 120.0; // inches per second

	public static double kPathFollowingProfileKp = 3.0;
	public static double kPathFollowingProfileKi = 0.03;
	public static double kPathFollowingProfileKv = 0.02;
	public static double kPathFollowingProfileKffv = 1.0;
	public static double kPathFollowingProfileKffa = 0.05;
	public static double kPathFollowingGoalPosTolerance = 2;
	public static double kPathFollowingGoalVelTolerance = 18.0;
	public static double kPathStopSteeringDistance = 9.0;

	// Goal tracker constants
	public static double kMaxGoalTrackAge = 1.0;
	public static double kMaxTrackerDistance = 18.0;
	public static double kCameraFrameRate = 30.0;
	public static double kTrackReportComparatorStablityWeight = 1.0;
	public static double kTrackReportComparatorAgeWeight = 1.0;

	// Pose of the camera frame w.r.t. the robot frame
	public static double kCameraXOffset = -3.3211;
	public static double kCameraYOffset = 0.0;
	public static double kCameraZOffset = 20.9;
	public static double kCameraPitchAngleDegrees = 29.56; // Measured on 4/26
	public static double kCameraYawAngleDegrees = 0.0;
	public static double kCameraDeadband = 0.0;

}

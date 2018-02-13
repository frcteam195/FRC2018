package org.usfirst.frc.team195.robot.Utilities;

public class Constants {
	public static final boolean TUNING_PIDS = true;
	public static final boolean DEBUG = false;
	public static final boolean REPORTING_ENABLED = true;
	public static final boolean REPORT_TO_DRIVERSTATION_INSTEAD_OF_CONSOLE = false;

	public static final boolean ENABLE_DRIVE_DIAG = false;
	public static final boolean ENABLE_CUBE_HANDLER_DIAG = true;

	public static final String DASHBOARD_IP = "10.1.95.14";
	public static final int DASHBOARD_REPORTER_PORT = 5801;


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


	//Arm Config Attack3D
	public static final int ARM_MANUAL_POSITION_CONTROL = 2;









	
	public static final int kTimeoutMs = 20;
	public static final int kTimeoutMsFast = 10;
	public static final int kTalonRetryCount = 3;
	public static final double kJoystickDeadband = 0.02;
	public static final double kSensorUnitsPerRotation = 4096.0;
	public static final double k100msPerMinute = 600.0;
	public static final double kLooperDt = 0.005;


	/* ROBOT PHYSICAL CONSTANTS */

	// Wheels
	public static final double kDriveWheelDiameterInches = 4.875;
	public static final double kTrackWidthInches = 23.75;
	public static final double kTrackScrubFactor = 0.924; // 0.924 ?

	// Geometry
	public static final double kCenterToFrontBumperDistance = 14.5;
	public static final double kCenterToIntakeDistance = 23.11;
	public static final double kCenterToRearBumperDistance = 14.5;
	public static final double kCenterToSideBumperDistance = 14.5;


	// Arm
	public static final double kArm1EncoderGearRatio = 10.0;
	public static final double kArm1Length = 8.25;
	public static final double kArm1SoftMin = 0;	//In rotations of output shaft
	public static final double kArm1SoftMax = 0.5;	//In rotations of output shaft
	public static final int kArm1MaxContinuousCurrentLimit = 35;
	public static final int kArm1MaxPeakCurrentLimit = 45;
	public static final int kArm1MaxPeakCurrentDurationMS = 400;

	public static final double kArm2EncoderGearRatio = 10.0;
	public static final double kArm2Length = 8.5;
	public static final double kArm2SoftMin = -0.456348;	//In rotations of output shaft
	public static final double kArm2SoftMax = 0.456348;	//In rotations of output shaft
	public static final int kArm2MaxContinuousCurrentLimit = 35;
	public static final int kArm2MaxPeakCurrentLimit = 45;
	public static final int kArm2MaxPeakCurrentDurationMS = 400;

	public static final double kArmMinRadius = 0;
	public static final double kArmMaxRadius = kArm1Length + kArm2Length;
	public static final double kArmMinTheta = 0;
	public static final double kArmMaxTheta = 180;
	public static final double kArmJoystickInchesPerSec = 4;
	public static final double kArmJoystickDegPerSec = 15;

	// Elevator
	public static final double kElevatorEncoderGearRatio = 1.0;
	public static final double kElevatorSoftMin = 0;	//In rotations of output shaft
	public static final double kElevatorSoftMax = 4;	//In rotations of output shaft
	public static final int kElevatorMaxContinuousCurrentLimit = 25;
	public static final int kElevatorMaxPeakCurrentLimit = 45;
	public static final int kElevatorMaxPeakCurrentDurationMS = 400;




	//TODO: Tune collision detection
	// Collision Detection
	public static final double kCollisionDetectionJerkThreshold = 0.5;


	/* CONTROL LOOP GAINS */

	//TODO: Tune drive base gains for velocity control
	// PID gains for drive velocity loop (HIGH GEAR)
	// Units: setpoint, error, and output are in inches per second.
	public static final double kDriveHighGearVelocityKp = 0.43;
	public static final double kDriveHighGearVelocityKi = 0.4;
	public static final double kDriveHighGearVelocityKd = 5.0;
	public static final double kDriveHighGearVelocityKf = 0.385;
	public static final int kDriveHighGearVelocityIZone = 15;
	public static final double kDriveHighGearVelocityRampRate = 0.25;
	public static final double kDriveHighGearMaxSetpoint = 14.0 * 12.0; // 14 fps

	// PID gains for drive velocity loop (LOW GEAR)
	// Units: setpoint, error, and output are in inches per second.
	public static final double kDriveLowGearPositionKp = 0.43;
	public static final double kDriveLowGearPositionKi = 0.4;
	public static final double kDriveLowGearPositionKd = 5.0;
	public static final double kDriveLowGearPositionKf = .385;
	public static final int kDriveLowGearPositionIZone = 15;
	public static final double kDriveLowGearPositionRampRate = 0.25; // V/s
	public static final double kDriveLowGearMaxVelocity = 14.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 14 fps, value in RPM
	public static final double kDriveLowGearMaxAccel = 20.0 * 12.0 * 60.0 / (Math.PI * kDriveWheelDiameterInches); // 18 fps/s, value in RPM/s

	//TODO: Retune Arm Gains when arm is properly mounted
	public static final double kArm1Kp = 1;
	public static final double kArm1Ki = 0.006;
	public static final double kArm1Kd = 4;
	public static final double kArm1Kf = 0.8;
	public static final int kArm1IZone = 10;
	public static final double kArm1RampRate = 0;
	public static final int kArm1MaxVelocity = 300;
	public static final int kArm1MaxAccel = 250;

	public static final double kArm2Kp = 1;
	public static final double kArm2Ki = 0.006;
	public static final double kArm2Kd = 4;
	public static final double kArm2Kf = 0.8;
	public static final int kArm2IZone = 10;
	public static final double kArm2RampRate = 0;
	public static final int kArm2MaxVelocity = 700;
	public static final int kArm2MaxAccel = 450;

	//TODO: Tune Elevator Gains
	public static final double kElevatorKp = 1;
	public static final double kElevatorKi = 0.006;
	public static final double kElevatorKd = 4;
	public static final double kElevatorKf = 0.8;
	public static final int kElevatorIZone = 10;
	public static final double kElevatorRampRate = 0;
	public static final int kElevatorMaxVelocity = 700;
	public static final int kElevatorMaxAccel = 450;


	////////////////////////////////////////////////////////////////////////////////////
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

	// Arm
	public static final int kArm1MotorId = 7;
	public static final int kArm2MotorId = 8;

	// Intake
	public static final int kIntakeMotorId = 9;

	// Elevator
	public static final int kElevatorMasterId = 10;
	public static final int kElevatorSlaveId = 11;

	//Climber
	public static final int kClimberMasterId = 12;
	public static final int kClimberSlaveId = 13;



	////////////////////////////////////////////////////////////////////////////////////

	/* PDP Channel IDs */

	// Drive
	public static final int kLeftDriveMasterPDPChannel = 14;
	public static final int kLeftDriveSlave1PDPChannel = 13;
	public static final int kLeftDriveSlave2PDPChannel = 12;
	public static final int kRightDriveMasterPDPChannel = 1;
	public static final int kRightDriveSlave1PDPChannel = 2;
	public static final int kRightDriveSlave2PDPChannel = 3;

	// Arm
	public static final int kArm1MotorPDPChannel = 5;
	public static final int kArm2MotorPDPChannel = 6;

	// Intake
	public static final int kIntakeMotorPDPChannel = 10;

	// Elevator
	public static final int kElevatorMasterPDPChannel = 11;
	public static final int kElevatorSlavePDPChannel = 4;

	//Climber
	public static final int kClimberMasterPDPChannel = 0;
	public static final int kClimberSlavePDPChannel = 15;


	////////////////////////////////////////////////////////////////////////////////////

	//TODO: Get list of solenoid IDs and input
	// Solenoids
	public static final int kIntakeSolenoidId = 0; // PCM 0, Solenoid 0
	public static final int kClimberLockSolenoidId1 = 1; // PCM 0, Solenoid 0
	public static final int kClimberLockSolenoidId2 = 2; // PCM 0, Solenoid 0

	// Digital Outputs
	public static final int kRedLEDId = 0;
	public static final int kGreenLEDId = 1;
	public static final int kBlueLEDId = 2;

	//TODO: Tune path following gains
	// Path following constants
	public static final double kMinLookAhead = 12.0; // inches
	public static final double kMinLookAheadSpeed = 9.0; // inches per second
	public static final double kMaxLookAhead = 24.0; // inches
	public static final double kMaxLookAheadSpeed = 100.0; // inches per second
	public static final double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
	public static final double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

	public static final double kInertiaSteeringGain = 0.017; // angular velocity command is multiplied by this gain *
	// our speed
	// in inches per sec
	public static final double kSegmentCompletionTolerance = 1; // inches
	public static final double kPathFollowingMaxAccel = 100.0; // inches per second^2
	public static final double kPathFollowingMaxVel = 84.0; // inches per second

	public static final double kPathFollowingProfileKp = 3.0;
	public static final double kPathFollowingProfileKi = 0.03;
	public static final double kPathFollowingProfileKv = 0.2;
	public static final double kPathFollowingProfileKffv = 1.0;
	public static final double kPathFollowingProfileKffa = 0.05;
	public static final double kPathFollowingGoalPosTolerance = 1;
	public static final double kPathFollowingGoalVelTolerance = 18.0;
	public static final double kPathStopSteeringDistance = 9.0;



	////////NOT USED THIS YEAR


	// Turn to heading gains
	public static final double kDriveTurnKp = 3.0;
	public static final double kDriveTurnKi = 1.5;
	public static final double kDriveTurnKv = 0.0;
	public static final double kDriveTurnKffv = 1.0;
	public static final double kDriveTurnKffa = 0.0;
	public static final double kDriveTurnMaxVel = 360.0;
	public static final double kDriveTurnMaxAcc = 720.0;

	// Goal tracker constants
	public static final double kMaxGoalTrackAge = 1.0;
	public static final double kMaxTrackerDistance = 18.0;
	public static final double kCameraFrameRate = 30.0;
	public static final double kTrackReportComparatorStablityWeight = 1.0;
	public static final double kTrackReportComparatorAgeWeight = 1.0;

	// Pose of the camera frame w.r.t. the robot frame
	public static final double kCameraXOffset = -3.3211;
	public static final double kCameraYOffset = 0.0;
	public static final double kCameraZOffset = 20.9;
	public static final double kCameraPitchAngleDegrees = 29.56; // Measured on 4/26
	public static final double kCameraYawAngleDegrees = 0.0;
	public static final double kCameraDeadband = 0.0;

	// Target parameters
	// Source of current values: https://firstfrc.blob.core.windows.net/frc2017/Manual/2017FRCGameSeasonManual.pdf
	// Section 3.13
	// ...and https://firstfrc.blob.core.windows.net/frc2017/Drawings/2017FieldComponents.pdf
	// Parts GE-17203-FLAT and GE-17371 (sheet 7)
	public static final double kBoilerTargetTopHeight = 88.0;
	public static final double kBoilerRadius = 7.5;

}

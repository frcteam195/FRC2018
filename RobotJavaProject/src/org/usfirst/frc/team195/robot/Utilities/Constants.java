package org.usfirst.frc.team195.robot.Utilities;

public class Constants {
	public static final boolean TUNING_PIDS = true;
	public static final boolean DEBUG = false;
	public static final boolean REPORTING_ENABLED = true;
	public static final boolean REPORT_TO_DRIVERSTATION_INSTEAD_OF_CONSOLE = false;

	public static final boolean ENABLE_DRIVE_DIAG = false;
	public static final boolean ENABLE_CUBE_HANDLER_DIAG = true;
	public static final boolean ENABLE_CLIMBER_DIAG = false;

	public static final String DASHBOARD_IP = "10.1.95.14";
	public static final int DASHBOARD_REPORTER_PORT = 5801;


	//Drive Config Attack 3
	public static final int DRIVE_X_AXIS = 0;
	public static final int DRIVE_Y_AXIS = 1;
	public static final int DRIVE_IMM_TURN = 8;
	public static final int DRIVE_HOLD_BRAKE = 9;
	
	//Drive Config F310
//	public static final int DRIVE_X_AXIS = 2;
//	public static final int DRIVE_Y_AXIS = 1;
//	public static final int DRIVE_IMM_TURN = 7;
//	public static final int DRIVE_HOLD_BRAKE = 8;

	//Arm Config Attack3D
	public static final int ARM_MANUAL_POSITION_CONTROL = 2;
	public static final int ARM_INTAKE_IN = 7;
	public static final int ARM_INTAKE_OUT = 9;
	public static final int ARM_INTAKE_CLAMP = 8;
	public static final int ARM_INTAKE_UNCLAMP = 10;






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
	public static final int kClimberMasterId = 13;
	public static final int kClimberSlaveId = 12;



	////////////////////////////////////////////////////////////////////////////////////

	/* PDP Channel IDs */

	// Drive
	public static final int kLeftDriveMasterPDPChannel = 14;
	public static final int kLeftDriveMasterPDPBreakerRating = 40;
	public static final int kLeftDriveSlave1PDPChannel = 13;
	public static final int kLeftDriveSlave1PDPBreakerRating = 40;
	public static final int kLeftDriveSlave2PDPChannel = 12;
	public static final int kLeftDriveSlave2PDPBreakerRating = 40;
	public static final int kRightDriveMasterPDPChannel = 1;
	public static final int kRightDriveMasterPDPBreakerRating = 40;
	public static final int kRightDriveSlave1PDPChannel = 2;
	public static final int kRightDriveSlave1PDPBreakerRating = 40;
	public static final int kRightDriveSlave2PDPChannel = 3;
	public static final int kRightDriveSlave2PDPBreakerRating = 40;

	// Arm
	public static final int kArm1MotorPDPChannel = 5;
	public static final int kArm1MotorPDPBreakerRating = 30;
	public static final int kArm2MotorPDPChannel = 6;
	public static final int kArm2MotorPDPBreakerRating = 30;

	// Intake
	public static final int kIntakeMotorPDPChannel = 10;
	public static final int kIntakeMotorPDPBreakerRating = 30;

	// Elevator
	public static final int kElevatorMasterPDPChannel = 11;
	public static final int kElevatorMasterPDPBreakerRating = 30;
	public static final int kElevatorSlavePDPChannel = 4;
	public static final int kElevatorSlavePDPBreakerRating = 30;

	//Climber
	public static final int kClimberMasterPDPChannel = 0;
	public static final int kClimberMasterPDPBreakerRating = 40;
	public static final int kClimberSlavePDPChannel = 15;
	public static final int kClimberSlavePDPBreakerRating = 40;

	//Breaker model for trip time output in seconds y = a*(current_percent_over_rating)^b + c
	public static final double kPDPBreakerModelA = 282.2962;
	public static final double kPDPBreakerModelB = -6.6305;
	public static final double kPDPBreakerModelC = 0.5;
	public static final double kPDPDefaultSafetyFactor = 4.0;

	
	public static final int kTimeoutMs = 20;
	public static final int kTimeoutMsFast = 10;
	public static final int kTalonRetryCount = 3;
	public static final double kJoystickDeadband = 0.05;
	public static final double kSensorUnitsPerRotation = 4096.0;
	public static final double k100msPerMinute = 600.0;
	public static final double kLooperDt = 0.005;


	/* ROBOT PHYSICAL CONSTANTS */

	// Wheels
	public static final double kDriveWheelDiameterInches = 4.875;
	public static final double kTrackWidthInches = 23.75;
	public static final double kTrackScrubFactor = 1.0; // 0.924 ?

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
	public static final int kArm1MaxContinuousCurrentLimit = kArm1MotorPDPBreakerRating;
	public static final int kArm1MaxPeakCurrentLimit = kArm1MaxContinuousCurrentLimit * 2;
	public static final int kArm1MaxPeakCurrentDurationMS = getMSDurationForBreakerLimit(kArm1MaxPeakCurrentLimit, kArm1MaxContinuousCurrentLimit);

	public static final double kArm2EncoderGearRatio = 1.0;
	public static final double kArm2Length = 8.5;
	public static final double kArm2SoftMin = -0.456348;	//In rotations of output shaft
	public static final double kArm2SoftMax = 0.456348;	//In rotations of output shaft
	public static final int kArm2MaxContinuousCurrentLimit = kArm2MotorPDPBreakerRating;
	public static final int kArm2MaxPeakCurrentLimit = kArm2MaxContinuousCurrentLimit * 2;
	public static final int kArm2MaxPeakCurrentDurationMS = getMSDurationForBreakerLimit(kArm2MaxPeakCurrentLimit, kArm2MaxContinuousCurrentLimit);

	public static final double kArmMinRadius = 0;
	public static final double kArmMaxRadius = kArm1Length + kArm2Length;
	public static final double kArmMinTheta = 0;
	public static final double kArmMaxTheta = 180;
	public static final double kArmJoystickInchesPerSec = 32;
	public static final double kArmJoystickDegPerSec = 90;
	public static final double kArmJoystickYDeadband = 0.1;
	public static final double kArmJoystickZDeadband = 0.25;


	public static final int kIntakeMaxContinuousCurrentLimit = kIntakeMotorPDPBreakerRating;
	public static final int kIntakeMaxPeakCurrentLimit = kIntakeMaxContinuousCurrentLimit * 2;
	public static final int kIntakeMaxPeakCurrentDurationMS = getMSDurationForBreakerLimit(kIntakeMaxPeakCurrentLimit, kIntakeMaxContinuousCurrentLimit);

	// Elevator
	public static final double kElevatorEncoderGearRatio = 1.0;
	public static final double kElevatorSoftMin = 0;	//In rotations of output shaft
	public static final double kElevatorSoftMax = 4;	//In rotations of output shaft
	public static final int kElevatorMaxContinuousCurrentLimit = kElevatorMasterPDPBreakerRating;
	public static final int kElevatorMaxPeakCurrentLimit = kElevatorMaxContinuousCurrentLimit * 2;
	public static final int kElevatorMaxPeakCurrentDurationMS = getMSDurationForBreakerLimit(kElevatorMaxPeakCurrentLimit, kElevatorMaxContinuousCurrentLimit);;

	// Climber
	public static final double kClimberEncoderGearRatio = 1.0;
	public static final double kClimberSoftMin = 0;	//In rotations of output shaft
	public static final double kClimberSoftMax = 4;	//In rotations of output shaft
	public static final int kClimberMaxContinuousCurrentLimit = kClimberMasterPDPBreakerRating;
	public static final int kClimberMaxPeakCurrentLimit = kClimberMaxContinuousCurrentLimit * 2;
	public static final int kClimberMaxPeakCurrentDurationMS = getMSDurationForBreakerLimit(kClimberMaxPeakCurrentLimit, kClimberMaxContinuousCurrentLimit, 2);;


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
	public static final double kArm1Kf = 0.08;
	public static final int kArm1IZone = 10;
	public static final double kArm1RampRate = 0;
	public static final int kArm1MaxVelocity = 300;
	public static final int kArm1MaxAccel = 250;


	public static final double kArm2Kp = 10;
	public static final double kArm2Ki = 0.06;
	public static final double kArm2Kd = 40;
	public static final double kArm2Kf = 0.8;
	public static final int kArm2IZone = 10;
	public static final double kArm2RampRate = 0;
	public static final int kArm2MaxVelocity = 70;
	public static final int kArm2MaxAccel = 45;

	public static final double kIntakeKp = 0.2;
	public static final double kIntakeKi = 0;
	public static final double kIntakeKd = 0;
	public static final double kIntakeKf = 0.06;
	public static final int kIntakeIZone = 0;
	public static final double kIntakeRampRate = 0;

	//TODO: Tune Elevator Gains
	public static final double kElevatorKp = 1;
	public static final double kElevatorKi = 0.006;
	public static final double kElevatorKd = 4;
	public static final double kElevatorKf = 0.8;
	public static final int kElevatorIZone = 10;
	public static final double kElevatorRampRate = 0;
	public static final int kElevatorMaxVelocity = 700;
	public static final int kElevatorMaxAccel = 450;


	//TODO: Tune Climber Gains
	public static final double kClimberKp = 1;
	public static final double kClimberKi = 0.006;
	public static final double kClimberKd = 4;
	public static final double kClimberKf = 0.8;
	public static final int kClimberIZone = 10;
	public static final double kClimberRampRate = 0;
	public static final int kClimberMaxVelocity = 700;
	public static final int kClimberMaxAccel = 450;



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



	private static int getMSDurationForBreakerLimit(double peakCurrentInput, double breakerRating) {
		return getMSDurationForBreakerLimit(peakCurrentInput, breakerRating, kPDPDefaultSafetyFactor);
	}

	private static int getMSDurationForBreakerLimit(double peakCurrentInput, double breakerRating, double safetyFactor) {
		return (int)((kPDPBreakerModelA*Math.pow(peakCurrentInput/breakerRating, kPDPBreakerModelB)+kPDPBreakerModelC) * 1000.0 / safetyFactor);
	}



	//////////////////////////////////////
	//TEST CONSTRAINTS
	public static final double kDriveBaseTestLowCurrentThresh = 2;
	public static final double kDriveBaseTestLowRPMThresh = 100;
	public static final double kDriveBaseTestCurrentDelta = 5.0;
	public static final double kDriveBaseTestRPMDelta = 40.0;

	public static final double kArmTestLowCurrentThresh = 1;
	public static final double kArmTestLowRPMThresh = 35;
	public static final double kArmTestCurrentDelta = 5.0;
	public static final double kArm1TestSpeed = 0.3;
	public static final double kArm2TestSpeed = 0.3;
	public static final double kArm1TestDuration = 0.5;
	public static final double kArm2TestDuration = 0.75;

	public static final double kElevatorTestLowCurrentThresh = 2;
	public static final double kElevatorTestLowRPMThresh = 200;
	public static final double kElevatorTestCurrentDelta = 5.0;
	public static final double kElevatorTestRPMDelta = 40.0;
	public static final double kElevatorTestSpeed = 0.3;
	public static final double kElevatorTestDuration = 1;

	public static final double kIntakeTestLowCurrentThresh = 2;

	public static final double kClimberTestLowCurrentThresh = 2;
	public static final double kClimberTestLowRPMThresh = 35;
	public static final double kClimberTestCurrentDelta = 5.0;
	public static final double kClimberTestRPMDelta = 40.0;
	public static final double kClimberTestSpeed = 0.3;
	public static final double kClimberTestDuration = 1;




	//////////////////////////////////////




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

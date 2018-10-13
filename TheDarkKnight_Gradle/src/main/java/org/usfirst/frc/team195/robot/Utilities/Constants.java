package org.usfirst.frc.team195.robot.Utilities;

import org.usfirst.frc.team195.robot.Autonomous.Paths.PathAdapter;

public class Constants {
	public static final boolean TUNING_PIDS = false;
	public static final boolean DEBUG = false;
	public static final boolean REPORTING_ENABLED = true;
	public static final boolean REPORT_TO_DRIVERSTATION_INSTEAD_OF_CONSOLE = false;
	public static final RGBColor kDefaultColor = new RGBColor(210, 0, 120);  //Default purple color
	public static final RGBColor kCommLossColor = new RGBColor(255, 0, 0);
	public static final RGBColor kRequestCubeColor = new RGBColor(0, 255, 0);
	public static final RGBColor kGotCubeColor = kDefaultColor;
	public static final RGBColor kElevatorHomeColor = new RGBColor(0, 0, 255);


	public static final boolean ENABLE_DRIVE_DIAG = true;
	public static final boolean ENABLE_CUBE_HANDLER_DIAG = true;
	public static final boolean ENABLE_CLIMBER_DIAG = false;

	public static final String DASHBOARD_IP = "10.1.95.14";
	public static final int DASHBOARD_REPORTER_PORT = 5801;
	public static final int AUTO_SELECTOR_PORT = 5803;
	public static final int LOG_OSC_REPORTER_PORT = 5805;
	public static final int MOBILE_DIAGNOSTICS_PORT = 5807;

	//Thread prioritization - 5 is default
	public static final int kRobotThreadPriority = 9;
	public static final int kLooperThreadPriority = Thread.MAX_PRIORITY;
	public static final int kCriticalSystemsMonitorThreadPriority = 8;
	public static final int kConnectionMonitorThreadPriority = 7;
	public static final int kLEDThreadPriority = Thread.MIN_PRIORITY;
	public static final int kConsoleReporterThreadPriority = Thread.NORM_PRIORITY;
	public static final int kDashboardReporterThreadPriority = 6;


	//Drive Config Attack 3
	public static final int DRIVE_X_AXIS = 0;
	public static final int DRIVE_Y_AXIS = 1;
	public static final int DRIVE_IMM_TURN = 7;
	public static final int DRIVE_HOLD_BRAKE = 9;
	public static final int DRIVE_REQUEST_CUBE_FROM_WALL = 8;
	
	//Drive Config F310
//	public static final int DRIVE_X_AXIS = 2;
//	public static final int DRIVE_Y_AXIS = 1;
//	public static final int DRIVE_IMM_TURN = 7;
//	public static final int DRIVE_HOLD_BRAKE = 8;

	//Arm Config Attack3D
	public static final int ARM_Y_AXIS = 1;
	public static final int ARM_INTAKE_IN = 1;
	public static final int ARM_INTAKE_CLAMP = 3;
	public static final int ARM_INTAKE_UNCLAMP = 4;
	public static final int ARM_INTAKE_OUT = 5;
	public static final int ARM_INTAKE_OUT_HALFSPEED = 6;
	public static final int ARM_ARM_VERTICAL = 2;
	public static final int ARM_ELEVATOR_INCREMENT_POV = 0;
	public static final int ARM_ELEVATOR_DECREMENT_POV = 180;
	public static final int ARM_ARM_LOW_POV = 270;
	public static final int ARM_ARM_MID_POV = 90;


	public static final int BB1_ELEVATOR_HOME = 1;
	public static final int BB1_ELEVATOR_INCREMENT = 2;
	public static final int BB1_ELEVATOR_DECREMENT = 12;
	public static final int BB1_ELEVATOR_SWITCH = 3;
	public static final int BB1_ELEVATOR_SCALE = 4;
	public static final int BB1_ELEVATOR_SCALE_HIGH = 6;
	public static final int BB1_ELEVATOR_REHOME = 16;
	public static final int BB1_ARM_DOWN = 7;
	public static final int BB1_ARM_BACK = 13;
	public static final int BB1_ARM_SWITCH = 8;
	public static final int BB1_AUTO_SWITCH = 2;
	public static final int BB1_ELEVATOR_LOW = 9;
	public static final int BB1_ELEVATOR_MID = 10;
	public static final int BB1_ELEVATOR_HIGH = 11;
	public static final int BB1_ELEVATOR_OVER_BACK_LOW = 3;
	public static final int BB1_ELEVATOR_OVER_BACK_MID = 4;
	public static final int BB1_ELEVATOR_OVER_BACK_HIGH = 5;
	public static final int BB1_REQUEST_CUBE_FROM_WALL = 15;

	public static final int BB2_ARM_SET_ZERO = 1;
	public static final int BB2_ARM_SET_MANUAL = 2;
	public static final int BB2_CLIMBER_DEPLOY_PLATFORM = 5;
	public static final int BB2_CLIMBER_CLIMB_ROLL_DEPLOY = 6;
	public static final int BB2_CLIMBER_CLIMB_IN = 7;
	public static final int BB2_CLIMBER_CLIMB_HOOK_SLOW = 8;
	public static final int BB2_ELEVATOR_CALCULATED_SCALE = 9;
	public static final int BB2_ELEVATOR_OPEN_LOOP_UP_RANDY = 10;
	public static final int BB2_ELEVATOR_OPEN_LOOP_DOWN_RANDY = 11;

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
	public static final int kArmMotorId = 7;

	// Intake
	public static final int kIntakeMotorId = 9;
	public static final int kIntake2MotorId = 8;

	// Elevator
	public static final int kElevatorMasterId = 10;
	public static final int kElevatorSlaveId = 11;
	public static final int kElevatorSlave2Id = 14;
	public static final int kElevatorSlave3Id = 15;

	//Climber
	public static final int kClimberMasterId = 13;
	public static final int kClimberSlaveId = 12;

	//CANifier
	public static final int kCANifierLEDId = 30;



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
	public static final int kArmMotorPDPChannel = 5;
	public static final int kArmMotorPDPBreakerRating = 30;

	// Intake
	public static final int kIntakeMotorPDPChannel = 10;
	public static final int kIntakeMotorPDPBreakerRating = 30;
	public static final int kIntake2MotorPDPChannel = 6;
	public static final int kIntake2MotorPDPBreakerRating = 30;

	// Elevator
	public static final int kElevatorMasterPDPChannel = 11;
	public static final int kElevatorMasterPDPBreakerRating = 30;
	public static final int kElevatorSlavePDPChannel = 4;
	public static final int kElevatorSlavePDPBreakerRating = 30;
	public static final int kElevatorSlave2PDPChannel = 5;
	public static final int kElevatorSlave2PDPBreakerRating = 30;
	public static final int kElevatorSlave3PDPChannel = 6;
	public static final int kElevatorSlave3PDPBreakerRating = 30;

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
	public static final int kActionTimeoutS = 2;
	public static final int kTalonRetryCount = 3;
	public static final double kJoystickDeadband = 0.08;
	public static final double kWheelDeadband = 0.05;
	public static final double kSensorUnitsPerRotation = 4096.0;
	public static final double k100msPerMinute = 600.0;
	public static final double kLooperDt = 0.005;


	/* ROBOT PHYSICAL CONSTANTS */

	// Wheels
	//public static final double kDriveWheelDiameterInches = 4.875;	//Practice bot calibrated 4.875
	//public static final double kDriveWheelDiameterInches = 5;	//Comp bot measured val
	public static final double kDriveWheelDiameterInches = PathAdapter.getAdaptedWheelDiameter();
	public static final double kTrackWidthInches = 25.5;
	public static final double kTrackScrubFactor = 1.0; // 0.924 ?

	// Geometry
	public static final double kCenterToFrontBumperDistance = 18.75;
	public static final double kCenterToIntakeDistance = 18.75;
	public static final double kCenterToRearBumperDistance = 18.75;
	public static final double kCenterToSideBumperDistance = 16.375;

	// Arm
	public static final double kArmEncoderGearRatio = 1.0;
	public static final double kArmMotorPulley = 18.0;	//Teeth
	public static final double kArmArmPulley = 42.0;	//Teeth
	public static final double kArmFinalRotationsPerDegree = kArmArmPulley/kArmMotorPulley/360.0;
	public static final double kArmSoftMin = 0 * kArmFinalRotationsPerDegree;	//Number in degrees of arm converted to rotations
	public static final double kArmSoftMax = 175 * kArmFinalRotationsPerDegree; //Number in degrees of arm converted to rotations
	public static final double kArmHomingTimeout = 2;	//In seconds
	public static final double kArmHomingSpeed = 0.3;	//In PercentOutput
	public static final double kArmHomingSetpoint = 86.3 * kArmFinalRotationsPerDegree;	//Number in degrees of arm converted to rotations
	public static final double kArmDeviationThresholdDeg = 5;	//In degrees, used to be 3
	public static final int kArmMaxContinuousCurrentLimit = kArmMotorPDPBreakerRating;
	public static final int kArmMaxPeakCurrentLimit = kArmMaxContinuousCurrentLimit * 2;
	public static final int kArmMaxPeakCurrentDurationMS = getMSDurationForBreakerLimit(kArmMaxPeakCurrentLimit, kArmMaxContinuousCurrentLimit);


	public static final int kIntakeMaxContinuousCurrentLimit = kIntakeMotorPDPBreakerRating;
	public static final int kIntakeMaxPeakCurrentLimit = kIntakeMaxContinuousCurrentLimit * 2;
	public static final int kIntakeMaxPeakCurrentDurationMS = getMSDurationForBreakerLimit(kIntakeMaxPeakCurrentLimit, kIntakeMaxContinuousCurrentLimit);

	// Elevator
	public static final double kElevatorEncoderGearRatio = 1.0;
	public static final double kElevatorSoftMin = -10;	//In rotations of output shaft
	public static final double kElevatorHome = 0;	//In rotations of output shaft
	public static final double kElevatorSoftMax = 20;	//In rotations of output shaft
	public static final double kElevatorStepSize = 0.2;	//In rotations of output shaft
	public static final double kElevatorDeviationThreshold = 0.1;	//In rotations of output shaft
	public static final double kElevatorSetDeviationThreshold = 0.2;	//In rotations of output shaft
	public static final double kElevatorHomingSpeed = -0.3;	//In percent output
	public static final double kElevatorSafetyCurrent = 50;	//In amps
	public static final double kElevatorSafetyDelta = 0.05;	//In rotations of output shaft
	public static final double kElevatorHomingTimeout = 2.5;	//In seconds
	public static final int kElevatorMaxContinuousCurrentLimit = kElevatorMasterPDPBreakerRating;
	public static final int kElevatorMaxPeakCurrentLimit = (int)(kElevatorMaxContinuousCurrentLimit * 1.5);
	public static final int kElevatorMaxPeakCurrentDurationMS = getMSDurationForBreakerLimit(kElevatorMaxPeakCurrentLimit, kElevatorMaxContinuousCurrentLimit, 2);;
	public static final double kElevatorRetensioningTime = 0.4;

	// Climber
	public static final double kClimberEncoderGearRatio = 1.0;
	public static final double kClimberSoftMin = 0;	//In rotations of output shaft
	public static final double kClimberSoftMax = 10;	//In rotations of output shaft
	public static final int kClimberMaxContinuousCurrentLimit = kClimberMasterPDPBreakerRating;
	public static final int kClimberMaxPeakCurrentLimit = kClimberMaxContinuousCurrentLimit * 2;
	public static final int kClimberMaxPeakCurrentDurationMS = getMSDurationForBreakerLimit(kClimberMaxPeakCurrentLimit, kClimberMaxContinuousCurrentLimit, 2);;


	//TODO: Tune collision detection
	// Collision Detection
	public static final double kCollisionDetectionJerkThreshold = 950;
	public static final double kTippingThresholdDeg = 11;


	/* CONTROL LOOP GAINS */

	// PID gains for drive velocity loop (HIGH GEAR)
	// Units: setpoint, error, and output are in inches per second.
	public static final double kDriveHighGearVelocityKp = 1;
	public static final double kDriveHighGearVelocityKi = 0.005;
	public static final double kDriveHighGearVelocityKd = 1.6;
	public static final double kDriveHighGearVelocityKf = 0.165;
	public static final int kDriveHighGearVelocityIZone = 0;
	public static final double kDriveHighGearVelocityRampRate = 0.1;
	public static final double kDriveHighGearMaxSetpoint = 12.0 * 12.0; // 12 fps

	// PID gains for drive velocity loop (LOW GEAR)
	// Units: setpoint, error, and output are in inches per second.
	public static final double kDriveLowGearPositionKp = 4;
	public static final double kDriveLowGearPositionKi = 0;
	public static final double kDriveLowGearPositionKd = 8;
	public static final double kDriveLowGearPositionKf = .19;
	public static final int kDriveLowGearPositionIZone = 0;
	public static final double kDriveLowGearPositionRampRate = 0; // V/s
	public static final double kDriveLowGearMaxVelocity = 712; // rpm
	public static final double kDriveLowGearMaxAccel = 500; // rpm/s

	//Tuned with 100:1 Transmission
	public static final double kArmKp = 6.7;
	public static final double kArmKi = 0;
	public static final double kArmKd = 11;
	public static final double kArmKf = 1;
	public static final int kArmIZone = 0;
	public static final double kArmRampRate = 0;
	public static final int kArmMaxVelocity = 450;
	public static final int kArmMaxAccel = 200;
	public static final int kArmMaxAccelDownFast = 350;	//350
	public static final int kArmAllowedError = (int)(0 * kSensorUnitsPerRotation);

	//Tuned for current control on 16:1 transmission
	public static final double kIntakeKp = 0.2;
	public static final double kIntakeKi = 0;
	public static final double kIntakeKd = 0;
	public static final double kIntakeKf = 0.06;
	public static final int kIntakeIZone = 0;
	public static final double kIntakeCLRampRate = 0.1;
	public static final double kIntakeOLRampRate = 0.1;
	public static final double kIntakeHoldCurrent = 2;

	//Tuned with 16:1 Transmission
	public static final double kElevatorKp = 1.6;
	public static final double kElevatorKi = 0;
	public static final double kElevatorKd = 3;
	public static final double kElevatorKf = 0.1638398438;
	public static final int kElevatorIZone = 0;
	public static final double kElevatorRampRate = 0;
	public static final int kElevatorMaxVelocityUp = 950; //Old value 950, fast 1200
	public static final int kElevatorMaxAccelUp = 1600;	//Old value was 1600, fast 1900
	public static final int kElevatorMaxVelocityDown = 1200;	//1200
	public static final int kElevatorMaxAccelDown = 5500;	//5500
	public static final double kElevatorInchesPerRotation = 5;
	public static final double kElevatorInchAdditionOffset = 6;

//	Old gains for climber motor
//	public static final double kClimberKp = 1;
//	public static final double kClimberKi = 0.006;
//	public static final double kClimberKd = 4;
//	public static final double kClimberKf = 0.966796875;
//	public static final int kClimberIZone = 60;
//	public static final double kClimberRampRate = 0.5;
//	public static final int kClimberMaxVelocity = 155;
//	public static final int kClimberMaxAccel = 340;

	public static final double kClimberKp = 1;
	public static final double kClimberKi = 0.01;
	public static final double kClimberKd = 4;
	public static final double kClimberKf = 0.75;
	public static final int kClimberIZone = 60;
	public static final double kClimberRampRate = 0.5;
	public static final double kClimberHoldRampRate = 0;
	public static final int kClimberMaxVelocity = 190;
	public static final int kClimberMaxAccel = 340;


	////////////////////////////////////////////////////////////////////////////////////

	// Solenoids
	public static final int kIntakeSolenoidId = 0; // PCM 0, Solenoid 0
	public static final int kClimberLockSolenoidId = 1; // PCM 0, Solenoid 0

	//Digital Inputs
	public static final int kElevatorHomeSwitchId = 0;
	public static final int kCubeSensorId = 1;

	// Digital Outputs
//	public static final int kRedLEDId = 2;
//	public static final int kGreenLEDId = 3;
//	public static final int kBlueLEDId = 4;

	// Path following constants
	public static final double kMinLookAhead = 12.0; // inches
	public static final double kMinLookAheadSpeed = 9.0; // inches per second
	public static final double kMaxLookAhead = 24.0; // inches
	public static final double kMaxLookAheadSpeed = 140.0; // inches per second
	public static final double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
	public static final double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

	public static final double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
	// our speed
	// in inches per sec
	public static final double kSegmentCompletionTolerance = 1; // inches
	public static final double kPathFollowingMaxAccel = 100.0; // inches per second^2
	public static final double kPathFollowingMaxVel = 140.0; // inches per second

	public static final double kPathFollowingProfileKp = 5.0;   //Used to be 5 when tuning our paths
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
	public static final double kArmTestSpeed = 0.3;
	public static final double kArmTestDuration = 0.5;

	public static final double kElevatorTestLowCurrentThresh = 2;
	public static final double kElevatorTestLowRPMThresh = 15;
	public static final double kElevatorTestCurrentDelta = 5.0;
	public static final double kElevatorTestRPMDelta = 10.0;
	public static final double kElevatorTestSpeed = 0.75;
	public static final double kElevatorTestDuration = 1;

	public static final double kIntakeTestLowCurrentThresh = 2;
	public static final double kIntakeTestSpeed = 1;
	public static final double kIntakeTestDuration = 2;

	public static final double kClimberTestLowCurrentThresh = 2;
	public static final double kClimberTestLowRPMThresh = 35;
	public static final double kClimberTestCurrentDelta = 5.0;
	public static final double kClimberTestRPMDelta = 40.0;
	public static final double kClimberTestSpeed = 0.3;
	public static final double kClimberTestDuration = 1;



	//////////////////////////////////////
	//FIRST POWER UP CONFIG DATA - Measurements in Inches
	private static final double kScaleArmTotalLength = 15 * 12;
	private static final double kScalePlateLength = 3 * 12;
	private static final double kScalePlateCenterWallHeight = 3.5;
	public static final double kScaleLevelHeight = 5 * 12 + kScalePlateCenterWallHeight;
	public static final double kScaleMinHeight = 4 * 12 + kScalePlateCenterWallHeight;
	public static final double kScaleMaxHeight = 6 * 12 + kScalePlateCenterWallHeight;
	public static final double kScaleMinAngle = Math.toDegrees(Math.tan((kScaleMinHeight - kScaleLevelHeight)/(kScaleArmTotalLength/2.0)));
	public static final double kScaleMaxAngle = Math.toDegrees(Math.tan((kScaleMaxHeight - kScaleLevelHeight)/(kScaleArmTotalLength/2.0)));
	public static final double kScaleArmCenterToPlateCenter = kScaleArmTotalLength / 2.0 - (kScalePlateLength / 2.0);	//Length to midpoint of scale plate from midpoint of scale arm
	//////////////////////////////////////




	////////NOT USED THIS YEAR


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

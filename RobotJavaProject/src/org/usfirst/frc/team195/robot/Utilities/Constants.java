package org.usfirst.frc.team195.robot.Utilities;

public class Constants {
	public static final boolean TUNING_PIDS = true;
	public static final boolean DEBUG = false;
	public static final boolean REPORTING_ENABLED = true;
	public static final boolean REPORT_TO_DRIVERSTATION_INSTEAD_OF_CONSOLE = false;


//	//SplineMotion trajectory constants
//	@Deprecated
//	public static final SRXTrajectoryConfig SXC = new SRXTrajectoryConfig();
//	static {
//		SXC.name = "StandardConfig";
//		SXC.dt = .01;   //Time step in seconds
//		SXC.max_acc = 30;   //Inches per sec^2
//		SXC.max_jerk = 720.0;   //Inches per sec^3
//		SXC.max_vel = 54.0; //Inches per sec
//		SXC.wheelbaseWidthInches = 24.5;    //Inches
//		SXC.wheelDiameterInches = 4.88; //Inches
//		SXC.encoderRotToWheelRotFactor = 1; //Conversion from encoder rotations to wheel rotations
//		SXC.encoderTicksPerRev = 4096;  //Encoder ticks per revolution of encoder
//	}




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
	public static final double kSensorUnitsPerRotation = 4096.0;
	public static final double k100msPerMinute = 600.0;




	// Target parameters
	// Source of current values: https://firstfrc.blob.core.windows.net/frc2017/Manual/2017FRCGameSeasonManual.pdf
	// Section 3.13
	// ...and https://firstfrc.blob.core.windows.net/frc2017/Drawings/2017FieldComponents.pdf
	// Parts GE-17203-FLAT and GE-17371 (sheet 7)
	public static double kBoilerTargetTopHeight = 88.0;
	public static double kBoilerRadius = 7.5;


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

	// Intake
	public static final int kIntakeLeftId = 7;
	public static final int kIntakeRightId = 8;

	// Elevator
	public static final int kElevatorMasterId = 9;
	public static final int kElevatorSlaveId = 10;

	// Arm
	public static final int kShoulderMotorId = 11;
	public static final int kElbowMotorId = 12;

	////////////////////////////////////////////////////////////////////////////////////

	// Solenoids
	public static final int kShifterSolenoidId = 1; // PCM 0, Solenoid 0
	public static final int kShifterSolenoidId2 = 0; // PCM 0, Solenoid 0

	// Digital Outputs
	public static int kLEDId = 0;

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

package org.usfirst.frc.team195.robot.Utilities;

import org.usfirst.frc.team195.robot.Utilities.Motion.SRX.SRXTrajectoryConfig;

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
}

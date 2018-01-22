package org.usfirst.frc.team195.robot.Utilities;

public class Constants {
	public static final boolean TUNING_PIDS = false;
	public static final boolean DEBUG = false;
	public static final boolean REPORTING_ENABLED = true;
	public static final boolean REPORT_TO_DRIVERSTATION_INSTEAD_OF_CONSOLE = false;

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

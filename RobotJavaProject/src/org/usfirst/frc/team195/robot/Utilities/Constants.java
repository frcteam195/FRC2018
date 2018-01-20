package org.usfirst.frc.team195.robot.Utilities;

public class Constants {
	public static final boolean TUNING_PIDS = false;
	public static final boolean DEBUG = false;

	//Drive Config
	public static final int DRIVE_X_AXIS = 0;
	public static final int DRIVE_Y_AXIS = 1;
	public static final int DRIVE_IMM_TURN = 8;
	public static final int DRIVE_SHIFT_LOW = 7;
	public static final int DRIVE_SHIFT_HIGH = 6;
	
	public static final int kTimeoutMs = 10;
	public static final double kMotorDeadband = 0.01;
	public static final double kSensorUnitsPerRotation = 4096;
}

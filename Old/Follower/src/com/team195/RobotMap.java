package com.team195;

public class RobotMap {
	public static final int DRIVE_JOYSTICK_PORT = 0;
	public static final int DRIVE_JOYSTICK_2_PORT = 1;

	public static final int LEFT_DRIVE_1 = 1;
	public static final int LEFT_DRIVE_2 = 2;
	public static final int LEFT_DRIVE_3 = 3;
	public static final int RIGHT_DRIVE_1 = 4;
	public static final int RIGHT_DRIVE_2 = 5;
	public static final int RIGHT_DRIVE_3 = 6;

	public static final boolean REVERSE_LEFT_DRIVE = false;
	public static final boolean REVERSE_RIGHT_DRIVE = true;

	public static final int SHIFT_SOL_REVERSE = 0;
	public static final int SHIFT_SOL_FORWARD = 1;

	public static final int UDP_PORT = 5801;
	public static final String UDP_IP = "10.1.95.21";
	public static final int PACKET_LENGTH = 1024;

	public static final double DESIRED_DISTANCE = 8;
	public static final double allowedDeviation = 2;
	public static final double allowedDistanceError = 0.5;


	public static final double ANGLE_PID_KP = 1.6;
	public static final double ANGLE_PID_KI = 0;
	public static final double ANGLE_PID_KD = 0.3;

	public static final double DRIVE_PID_KP = 4.6;
	public static final double DRIVE_PID_KI = 0;
	public static final double DRIVE_PID_KD = 0.5;

}

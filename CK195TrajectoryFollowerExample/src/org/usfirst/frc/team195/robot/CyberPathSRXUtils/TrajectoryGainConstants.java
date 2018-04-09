package org.usfirst.frc.team195.robot.CyberPathSRXUtils;

public class TrajectoryGainConstants {
	////////////////////////////////////////////////
	//Gains for motion profiling PID controls

	public static final int driveProfileSlot = 0;
	public static final int turnProfileSlot = 1;

	public static final double drivekP = 2;
	public static final double drivekI = 0;
	public static final double drivekD = 1;
	public static final double driveF = 0.36;
	public static final int driveIZone = 0;

	public static final double turnkP = 2.5;
	public static final double turnkI = 0.005;
	public static final double turnkD = 10;
	public static final double turnF = 0.2;
	public static final int turnIZone = 20;
	////////////////////////////////////////////////
}

package org.usfirst.frc.team195.robot.Utilities;

public class DriveMotorValues {
	public double leftDrive;
	public double rightDrive;
	
	public DriveMotorValues(double leftDrive, double rightDrive) {
		this.leftDrive = leftDrive;
		this.rightDrive = rightDrive;
	}

	public static DriveMotorValues NEUTRAL = new DriveMotorValues(0, 0);
}

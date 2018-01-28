package org.usfirst.frc.team195.robot.Utilities.Motion.SRX;

public class SRXDriveBaseTrajectory {
	private double[][] leftWheelTrajectory;
	private double[][] rightWheelTrajectory;

	public SRXDriveBaseTrajectory(double[][] leftWheelTrajectory, double[][] rightWheelTrajectory) {
		this.leftWheelTrajectory = leftWheelTrajectory;
		this.rightWheelTrajectory = rightWheelTrajectory;
	}

	public double[][] getLeftWheelTrajectory() {
		return leftWheelTrajectory;
	}

	public double[][] getRightWheelTrajectory() {
		return rightWheelTrajectory;
	}
}

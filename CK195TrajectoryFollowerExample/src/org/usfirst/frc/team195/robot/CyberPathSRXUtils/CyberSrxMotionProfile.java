package org.usfirst.frc.team195.robot.CyberPathSRXUtils;

//Generic Motion Profile Class
public class CyberSrxMotionProfile {

	public int numPoints;
	// Position (rotations) Velocity (RPM) Duration (ms)
	public double[][] points;

	public CyberSrxMotionProfile() {
	}

	public CyberSrxMotionProfile(int numPoints, double[][] points) {
		this.numPoints = numPoints;
		this.points = points;
	}
}

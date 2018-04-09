package org.usfirst.frc.team195.robot.CyberPathSRXUtils;

//Combines left and right motion profiles in one object
public class CyberSRXTrajectory {
	public boolean flipped;
	public boolean highGear;
	public CyberSrxMotionProfile profile;

	public CyberSRXTrajectory() {
	}

	public CyberSRXTrajectory(CyberSrxMotionProfile profile) {
		this.profile = profile;
	}
}

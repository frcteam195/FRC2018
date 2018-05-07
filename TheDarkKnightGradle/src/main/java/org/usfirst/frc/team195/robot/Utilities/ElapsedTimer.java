package org.usfirst.frc.team195.robot.Utilities;

import edu.wpi.first.wpilibj.Timer;

public class ElapsedTimer {
	private double startTime = 0;
	
	public ElapsedTimer() {
		
	}
	
	public void start() {
		startTime = Timer.getFPGATimestamp();
	}
	
	public double hasElapsed() {
		return Timer.getFPGATimestamp() - startTime;
	}
}

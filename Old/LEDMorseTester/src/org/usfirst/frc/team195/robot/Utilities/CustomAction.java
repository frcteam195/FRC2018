package org.usfirst.frc.team195.robot.Utilities;

public abstract class CustomAction extends Thread {
	public CustomAction() {
		running = false;

		timeoutStart = 0;
		timeoutEnd = 0;
		timeoutElapsedTimeMS = 0;
	}
	
	public boolean isRunning() {
		return running;
	}
	
	public abstract void run();
	
	protected double timeoutStart;
	protected double timeoutEnd;
	protected int timeoutElapsedTimeMS;
	protected boolean running;
}
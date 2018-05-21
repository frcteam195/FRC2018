package org.usfirst.frc.team195.robot.Utilities;

public class TimeoutTimer {
	private double timeout;
	private ElapsedTimer eTimer = new ElapsedTimer();
	private boolean firstRun;

	public TimeoutTimer(double timeout) {
		this.timeout = timeout;
		setFirstRun(true);
	}

	public boolean isTimedOut() {
		if (firstRun) {
			eTimer.start();
			setFirstRun(false);
		}
		return eTimer.hasElapsed() > timeout;
	}

	public void reset() {
		setFirstRun(true);
	}

	private synchronized void setFirstRun(boolean firstRun) {
		this.firstRun = firstRun;
	}
}

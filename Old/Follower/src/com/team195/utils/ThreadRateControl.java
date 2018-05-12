package com.team195.utils;


import edu.wpi.first.wpilibj.Timer;

public class ThreadRateControl {
	private double startTime;
	private double endTime;
	private int elapsedTimeMS;
	private boolean started;

	public ThreadRateControl() {
		startTime = 0;
		endTime = 0;
		elapsedTimeMS = 0;
		started = false;
	}

	public synchronized void start() {
		if (!started) {
			startTime = Timer.getFPGATimestamp();
			started = true;
		} else {

		}
	}

	public void doRateControl(int minLoopTime) {
		if (startTime != 0) {
			do {
				endTime = Timer.getFPGATimestamp();
				elapsedTimeMS = (int) ((endTime - startTime) * 1000);
				if (elapsedTimeMS < minLoopTime) {
					try {
						Thread.sleep(minLoopTime - elapsedTimeMS);
					} catch (Exception ex) {

					}
				}
			} while (elapsedTimeMS < minLoopTime);
		} else {

		}

		startTime = Timer.getFPGATimestamp();
	}
}
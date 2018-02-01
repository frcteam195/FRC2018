package org.usfirst.frc.team195.robot.Utilities;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;

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
			ConsoleReporter.report("Thread rate control start called too many times!", MessageLevel.ERROR);
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
						ConsoleReporter.report(ex);
					}
				}
			} while (elapsedTimeMS < minLoopTime);
		} else {
			ConsoleReporter.report("Thread rate control called without setting a start time! Check your loops!", MessageLevel.ERROR);
		}

		startTime = Timer.getFPGATimestamp();
	}
}

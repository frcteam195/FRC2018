package org.usfirst.frc.team195.robot;

import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.CriticalSystemStatus;
import org.usfirst.frc.team195.robot.Utilities.CustomSubsystem;
import org.usfirst.frc.team195.robot.Utilities.ThreadRateControl;

import java.util.ArrayList;

public class CriticalSystemsMonitor extends Thread {
	private static final int MIN_SYSTEM_MONITOR_LOOP_MS = 100;

	private static CriticalSystemsMonitor instance = null;

	private boolean runThread = true;
	private ThreadRateControl threadRateControl = new ThreadRateControl();
	private ArrayList<CriticalSystemStatus> mSystemArr;

	public CriticalSystemsMonitor(ArrayList<CustomSubsystem> subsystems) {
		mSystemArr = new ArrayList<CriticalSystemStatus>();
		for (CustomSubsystem cs : subsystems) {
			if (cs instanceof CriticalSystemStatus)
				mSystemArr.add((CriticalSystemStatus) cs);
		}
	}

	public static CriticalSystemsMonitor getInstance(ArrayList<CustomSubsystem> subsystems) {
		if(instance == null) {
			try {
				instance = new CriticalSystemsMonitor(subsystems);
			} catch (Exception ex) {
				ConsoleReporter.report(ex, MessageLevel.DEFCON1);
			}
		}

		return instance;
	}

	@Override
	public void start() {
		runThread = true;
		if (!super.isAlive())
			super.start();
	}

	@Override
	public void run() {
		threadRateControl.start();
		while (runThread) {
			for (CriticalSystemStatus css : mSystemArr) {
				css.isSystemFaulted();
			}
			threadRateControl.doRateControl(MIN_SYSTEM_MONITOR_LOOP_MS);
		}
	}
}

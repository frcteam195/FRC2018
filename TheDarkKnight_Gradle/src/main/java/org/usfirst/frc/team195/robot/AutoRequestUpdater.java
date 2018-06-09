package org.usfirst.frc.team195.robot;

import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Subsystems.CubeHandlerSubsystem;
import org.usfirst.frc.team195.robot.Utilities.ThreadRateControl;

public class AutoRequestUpdater {
	public static final int MIN_AUTO_RECEIVE_RATE_MS = 20;
	private static AutoRequestUpdater instance = null;

	private boolean runThread;
	private ThreadRateControl threadRateControl = new ThreadRateControl();
	private Thread t;

	private CubeHandlerSubsystem cubeHandlerSubsystem = CubeHandlerSubsystem.getInstance();
	private double armPosition = Double.MIN_VALUE;
	private double elevatorHeight = Double.MIN_VALUE;

	private AutoRequestUpdater() throws Exception {
		super();
		runThread = false;
	}

	public static AutoRequestUpdater getInstance() {
		if(instance == null) {
			try {
				instance = new AutoRequestUpdater();
			} catch (Exception ex) {
				ConsoleReporter.report(ex, MessageLevel.DEFCON1);
			}
		}

		return instance;
	}

	private synchronized void start() {
		if (t == null || !t.isAlive()) {
			setRunThread(true);

			t = new Thread(() -> {
				t.setPriority(Thread.NORM_PRIORITY);

				threadRateControl.start();
				while (runThread) {

					if (armPosition != Double.MIN_VALUE)
						cubeHandlerSubsystem.setArmRotationDeg(armPosition);

					if (elevatorHeight != Double.MIN_VALUE)
						cubeHandlerSubsystem.setElevatorHeight(elevatorHeight);

					threadRateControl.doRateControl(MIN_AUTO_RECEIVE_RATE_MS);

				}

			});

			t.start();
		}
	}

	public synchronized void updateElevatorHeightRequest(double elevatorHeight) {
		this.elevatorHeight = elevatorHeight;
		start();
	}

	public synchronized void updateArmPosition(double armPosition) {
		this.armPosition = armPosition;
		start();
	}

	public synchronized void terminate() {
		setRunThread(false);
	}

	private synchronized void setRunThread(boolean runThread) {
		this.runThread = runThread;
	}
}

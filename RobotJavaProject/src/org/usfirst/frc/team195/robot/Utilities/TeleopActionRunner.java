package org.usfirst.frc.team195.robot.Utilities;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Actions.Framework.Action;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;

public class TeleopActionRunner {
	private double m_update_rate = 1.0 / 50.0;    //20ms update rate
	private Action action;
	private boolean finished = false;
	private double timeout;
	private Thread t;

	/**
	 * Create an action runner with a timeout
	 * @param action Action to run
	 * @param timeout Timeout in seconds.
	 */
	public TeleopActionRunner(Action action, double timeout) {
		this.action = action;
		this.timeout = timeout;
	}

	public boolean isFinished() {
		return finished;
	}

	private void start() {
		double startTime = Timer.getFPGATimestamp();
		t = new Thread(() -> {
			//t.setPriority(Thread.NORM_PRIORITY);

			action.start();

			while (!action.isFinished() && (Timer.getFPGATimestamp() - startTime) > timeout) {
				action.update();
				int waitTime = (int) (m_update_rate * 1000.0);

				try {
					Thread.sleep(waitTime);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}

			action.done();
			finished = true;
		});
		t.start();
	}

	public boolean runAction() {
		return runAction(false);
	}

	public boolean runAction(boolean waitForCompletion) {
		if (t == null || !t.isAlive())
			start();

		if (waitForCompletion) {
			try {
				t.join((int) (timeout * 1000.0));
			} catch (InterruptedException ex) {
				ConsoleReporter.report(action.getClass().getSimpleName() + " Action did not complete in time!", MessageLevel.ERROR);
				ConsoleReporter.report(ex, MessageLevel.ERROR);
			}
		}

		return finished;
	}
}

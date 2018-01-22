package org.usfirst.frc.team195.robot.Reporters;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Utilities.Constants;

import java.util.LinkedList;
import java.util.concurrent.locks.ReentrantLock;

public class ConsoleReporter extends Thread {
	private static final int MIN_CONSOLE_SEND_RATE_MS = 250;

	private boolean runThread;

	private double consoleSendThreadControlStart;
	private double consoleSendThreadControlEnd;
	private int consoleSendThreadControlElapsedTimeMS;
	private static LinkedList<String> sendMessageQueue = new LinkedList<String>();
	private static ReentrantLock _reporterMutex = new ReentrantLock();

	private static ConsoleReporter instance = null;

	public ConsoleReporter() throws Exception {
		super();
		runThread = false;
		consoleSendThreadControlStart = 0;
		consoleSendThreadControlEnd = 0;
		consoleSendThreadControlElapsedTimeMS = 0;
	}

	public static ConsoleReporter getInstance() {
		if(instance == null) {
			try {
				instance = new ConsoleReporter();
			} catch (Exception ex) {
				ex.printStackTrace();
			}
		}

		return instance;
	}

	@Override
	public void start() {
		runThread = true;
		super.start();
	}

	public void terminate() {
		runThread = false;
	}

	@Override
	public void run() {
		while (runThread) {
			consoleSendThreadControlStart = Timer.getFPGATimestamp();
			try {
				_reporterMutex.lock();
				try {
					if (Constants.REPORTING_ENABLED) {
						while (sendMessageQueue.peek() != null) {
							if (Constants.REPORT_TO_DRIVERSTATION_INSTEAD_OF_CONSOLE)
								DriverStation.reportError(sendMessageQueue.poll(), false);
							else
								System.out.println(sendMessageQueue.poll());
						}
					}
				} finally {
					_reporterMutex.unlock();
				}
			} catch (Exception ex) {

			}
			do {
				consoleSendThreadControlEnd = Timer.getFPGATimestamp();
				consoleSendThreadControlElapsedTimeMS = (int) ((consoleSendThreadControlEnd - consoleSendThreadControlStart) * 1000);
				if (consoleSendThreadControlElapsedTimeMS < MIN_CONSOLE_SEND_RATE_MS)
					try{Thread.sleep(MIN_CONSOLE_SEND_RATE_MS - consoleSendThreadControlElapsedTimeMS);}catch(Exception ex) {};
			} while(consoleSendThreadControlElapsedTimeMS < MIN_CONSOLE_SEND_RATE_MS);
		}
	}

	public static void report(String message) {
		if (Constants.REPORTING_ENABLED) {
			_reporterMutex.lock();
			try {
				sendMessageQueue.add(message + "\n\r");
			} finally {
				_reporterMutex.unlock();
			}
		}
	}

}

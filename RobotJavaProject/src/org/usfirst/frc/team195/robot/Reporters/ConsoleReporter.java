package org.usfirst.frc.team195.robot.Reporters;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Utilities.Constants;

import java.util.LinkedList;
import java.util.concurrent.locks.ReentrantLock;

public class ConsoleReporter extends Thread {

	private static MessageLevel reportingLevel = MessageLevel.ERROR;

	private static final int MIN_CONSOLE_SEND_RATE_MS = 250;

	private boolean runThread;

	private double consoleSendThreadControlStart;
	private double consoleSendThreadControlEnd;
	private int consoleSendThreadControlElapsedTimeMS;
	private static LinkedList<CKMessage> sendMessageQueue = new LinkedList<CKMessage>();
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
							CKMessage ckm = sendMessageQueue.poll();
							if (ckm.messageLevel.ordinal() <= reportingLevel.ordinal()) {
								if (Constants.REPORT_TO_DRIVERSTATION_INSTEAD_OF_CONSOLE) {
									switch (ckm.messageLevel) {
										case DEFCON1:
										case ERROR:
											DriverStation.reportError(ckm.message, false);
											break;
										case WARNING:
										case INFO:
											DriverStation.reportWarning(ckm.message, false);
											break;
										default:
											break;
									}
								} else {
									System.out.println(ckm.message);
								}
							}
						}
					}
				} finally {
					_reporterMutex.unlock();
				}
			} catch (Exception ex) {
				ex.printStackTrace();
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
		report(message, MessageLevel.WARNING);
	}

	public static void setReportingLevel(MessageLevel messageLevel) {
		ConsoleReporter.reportingLevel = messageLevel;
	}

	public static void report(String message, MessageLevel msgLvl) {
		if (Constants.REPORTING_ENABLED) {
			_reporterMutex.lock();
			try {
				switch (msgLvl) {
					case DEFCON1:
						message = "DEFCON1: " + message;
						break;
					case ERROR:
						message = "ERROR: " + message;
						break;
					case WARNING:
						message = "WARNING: " + message;
						break;
					case INFO:
					default:
						break;
				}
				sendMessageQueue.add(new CKMessage(message + "\n\r", msgLvl));
			} finally {
				_reporterMutex.unlock();
			}
		}
	}

}

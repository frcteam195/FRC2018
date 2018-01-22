package org.usfirst.frc.team195.robot.Reporters;

import edu.wpi.first.wpilibj.Timer;

import java.util.concurrent.Semaphore;
import java.util.concurrent.locks.ReentrantLock;

public class ConsoleReporter extends Thread {
	private static final int MIN_CONSOLE_SEND_RATE_MS = 250;

	private boolean runThread;

	private double consoleSendThreadControlStart;
	private double consoleSendThreadControlEnd;
	private int consoleSendThreadControlElapsedTimeMS;
	private static String sendMessage;
	private static ReentrantLock _reporterMutex;

	private static ConsoleReporter instance = null;

	public ConsoleReporter() throws Exception {
		super();
		runThread = false;
		consoleSendThreadControlStart = 0;
		consoleSendThreadControlEnd = 0;
		consoleSendThreadControlElapsedTimeMS = 0;
		_reporterMutex = new ReentrantLock();
		sendMessage = "";
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
				if(!sendMessage.isEmpty()) {
					_reporterMutex.lock();
					try {
						System.out.println(sendMessage);
						sendMessage = "";
					} finally {
						_reporterMutex.unlock();
					}
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

	public static synchronized void report(String message) {
		_reporterMutex.lock();
		try {
			sendMessage += message + "\n\r";
		} finally {
			_reporterMutex.unlock();
		}
	}


}

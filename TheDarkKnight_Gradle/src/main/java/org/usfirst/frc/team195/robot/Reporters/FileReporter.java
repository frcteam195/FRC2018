package org.usfirst.frc.team195.robot.Reporters;

import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team195.robot.Utilities.Constants;
import org.usfirst.frc.team195.robot.Utilities.ThreadRateControl;

import java.io.FileWriter;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.Date;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.concurrent.locks.ReentrantLock;

public class FileReporter extends Thread {
	private static FileReporter instance = null;


	private static final int MIN_FILE_WRITE_RATE_MS = 250;
	private static MessageLevel reportingLevel = MessageLevel.ERROR;
	private static LinkedHashSet<CKMessage> sendMessageSet = new LinkedHashSet<>();
	private static ReentrantLock _reporterMutex = new ReentrantLock();
	private boolean runThread;
	private ThreadRateControl threadRateControl = new ThreadRateControl();

	private PrintWriter logWriter;

	private Thread logThread;

	public static FileReporter getInstance() {
		if(instance == null) {
			try {
				instance = new FileReporter();
			} catch (Exception ex) {
				ex.printStackTrace();
			}
		}

		return instance;
	}

	private FileReporter() {
		try {
			logWriter = new PrintWriter(new FileWriter("/home/lvuser/DataLog.txt", true));
			logWriter.print(new Date().toString());
			logWriter.println();
		} catch (Exception e) {

		}
	}

	public static void setReportingLevel(MessageLevel messageLevel) {
		FileReporter.reportingLevel = messageLevel;
	}

	public static void report(Throwable t) { report(t, MessageLevel.ERROR); }

	public static void report(Throwable t, MessageLevel messageLevel) {
		StringWriter s = new StringWriter();
		t.printStackTrace(new PrintWriter(s));
		report(s.toString(), messageLevel);
	}

	public static void report(Object message) {
		report(String.valueOf(message));
	}

	public static void report(Object message, MessageLevel msgLvl) {
		report(String.valueOf(message), msgLvl);
	}

	public static void report(String message) {
		report(message, MessageLevel.WARNING);
	}

	public static void report(String message, MessageLevel msgLvl) {
		if (msgLvl == MessageLevel.DEFCON1 || (Constants.REPORTING_ENABLED && (msgLvl.ordinal() <= reportingLevel.ordinal()))) {
			_reporterMutex.lock();
			try {
				sendMessageSet.add(new CKMessage(message, msgLvl));
			} finally {
				_reporterMutex.unlock();
			}
		}
	}

	@Override
	public void start() {
		runThread = true;
		threadRateControl.start();
		super.start();
	}

	public void terminate() {
		runThread = false;
	}

	@Override
	public void run() {
		while (!DriverStation.getInstance().isEnabled()) {
			threadRateControl.doRateControl(100);
		}

		while (runThread) {
			try {
				_reporterMutex.lock();
				try {
					for (Iterator<CKMessage> i = sendMessageSet.iterator(); i.hasNext();) {
						CKMessage ckm = i.next();
						if (ckm.messageLevel == MessageLevel.DEFCON1 || (Constants.REPORTING_ENABLED && (ckm.messageLevel.ordinal() <= reportingLevel.ordinal()))) {
							String s = ckm.toString(true);
							logWriter.println(s);
							i.remove();
						}
					}

					logWriter.flush();
				} finally {
					_reporterMutex.unlock();
				}
			} catch (Exception ex) {
				ex.printStackTrace();
			}

			threadRateControl.doRateControl(MIN_FILE_WRITE_RATE_MS);
		}

		logWriter.close();
	}

}

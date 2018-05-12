package org.usfirst.frc.team195.robot.Reporters;

import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team195.robot.Utilities.Constants;
import org.usfirst.frc.team195.robot.Utilities.ThreadRateControl;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.concurrent.locks.ReentrantLock;

/**
 * A class to report messages to the console or DriverStation. Messages with a level of DEFCON1 will always be reported
 * whether reporting is enabled or not and will be reported both to the console and the DriverStation.
 */
public class ConsoleReporter extends Thread {

	private static final int MIN_CONSOLE_SEND_RATE_MS = 250;
	private static MessageLevel reportingLevel = MessageLevel.ERROR;
	private static LinkedHashSet<CKMessage> sendMessageSet = new LinkedHashSet<>();
	private static ReentrantLock _reporterMutex = new ReentrantLock();
	private static ConsoleReporter instance = null;
	private boolean runThread;
	private ThreadRateControl threadRateControl = new ThreadRateControl();

	private ConsoleReporter() throws Exception {
		super();
		super.setPriority(Constants.kConsoleReporterThreadPriority);
		runThread = false;
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

	public static void setReportingLevel(MessageLevel messageLevel) {
		ConsoleReporter.reportingLevel = messageLevel;
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
		while (runThread) {
			try {
				_reporterMutex.lock();
				try {
					for (Iterator<CKMessage> i = sendMessageSet.iterator(); i.hasNext();) {
						CKMessage ckm = i.next();
						if (ckm.messageLevel == MessageLevel.DEFCON1 || (Constants.REPORTING_ENABLED && (ckm.messageLevel.ordinal() <= reportingLevel.ordinal()))) {
							String s = ckm.toString();
							if (Constants.REPORT_TO_DRIVERSTATION_INSTEAD_OF_CONSOLE) {
								switch (ckm.messageLevel) {
									case DEFCON1:
										System.out.println(s);
									case ERROR:
										DriverStation.reportError(s, false);
										break;
									case WARNING:
									case INFO:
										DriverStation.reportWarning(s, false);
										break;
									default:
										break;
								}
							} else {
								System.out.println(s);
								if (ckm.messageLevel == MessageLevel.DEFCON1)
									DriverStation.reportError(s, false);
							}
							i.remove();
						}
					}
				} finally {
					_reporterMutex.unlock();
				}
			} catch (Exception ex) {
				ex.printStackTrace();
			}

			threadRateControl.doRateControl(MIN_CONSOLE_SEND_RATE_MS);
		}
	}

}

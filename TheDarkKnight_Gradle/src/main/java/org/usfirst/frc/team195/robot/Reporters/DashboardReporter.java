package org.usfirst.frc.team195.robot.Reporters;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;

import org.usfirst.frc.team195.robot.Utilities.*;

public class DashboardReporter extends Thread {

	private static final int MIN_DASHBOARD_SEND_RATE_MS = 100;
	
	private static final int SEND_PORT = Constants.DASHBOARD_REPORTER_PORT;
	
	private static final String SEND_IP = Constants.DASHBOARD_IP;

	private InetAddress IPAddress;

    private DatagramSocket clientSocket;
    private byte[] sendData;
    private DatagramPacket sendPacket;

    private ThreadRateControl threadRateControl = new ThreadRateControl();
    
    private List<CustomSubsystem> subsystemList;
	
	private boolean runThread;
	
	private static DashboardReporter instance;

	private static HashSet<DiagnosticMessage> diagnosticMessages = new HashSet<>();
	private static ReentrantLock _reporterMutex = new ReentrantLock();

	public static DashboardReporter getInstance(List<CustomSubsystem> subsystemList) {
		if(instance == null) {
			try {
				if (subsystemList != null)
					instance = new DashboardReporter(subsystemList);
				else
					throw new Exception("Cannot create DashboardReporter when first instance subsystem list is null!");
			} catch (Exception ex) {
				ConsoleReporter.report(ex.toString(), MessageLevel.ERROR);
			}
		}
		
		return instance;
	}
	
	private DashboardReporter(List<CustomSubsystem> subsystemList) throws Exception {
		super();
		super.setPriority(Constants.kDashboardReporterThreadPriority);
		runThread = false;
        clientSocket = new DatagramSocket(SEND_PORT);

        IPAddress = InetAddress.getByName(SEND_IP);

        this.subsystemList = subsystemList;
	}
	
	@Override
	public void run() {
		if(clientSocket.isClosed() || clientSocket == null) {
			try {
				clientSocket = new DatagramSocket(SEND_PORT);
			} catch (SocketException e) {
				e.printStackTrace();
			}
		}
		ConsoleReporter.report("Entering Dashboard Thread!", MessageLevel.INFO);
		threadRateControl.start();
		threadRateControl.doRateControl(2000);	//Wait for init
		while (runThread) {
            try {
                sendData = createSendData();
//				FileReporter.report(sendData);
                sendPacket = new DatagramPacket(sendData, sendData.length, IPAddress, SEND_PORT);
                clientSocket.send(sendPacket);
            } catch (Exception ex) {
            	ConsoleReporter.report("Failed dashboard report send!", MessageLevel.ERROR);
				ConsoleReporter.report(ex);
	        }

			threadRateControl.doRateControl(MIN_DASHBOARD_SEND_RATE_MS);
        } 
		
		if(!clientSocket.isClosed())
			clientSocket.close();
	}

	public synchronized byte[] getSendData() {
		return sendData;
	}

	private byte[] createSendData() {
		StringBuilder stringBuilder = new StringBuilder();
		for (CustomSubsystem customSubsystem : subsystemList) {
			if (customSubsystem instanceof Reportable)
				stringBuilder.append(((Reportable) customSubsystem).generateReport());
		}

		stringBuilder.append("DiagnosticMessages:");

		_reporterMutex.lock();
		try {
			for (Iterator<DiagnosticMessage> i = diagnosticMessages.iterator(); i.hasNext(); ) {
				DiagnosticMessage m = i.next();

				if (!m.getMessage().isEmpty())
					stringBuilder.append(m.getMessage()).append("|");

				i.remove();
			}
		} finally {
			_reporterMutex.unlock();
		}

		stringBuilder.append(";");

		try {
			LogDataReporter.reportOSCData(stringBuilder.toString());
		} catch (Exception e) {
//				e.printStackTrace();
		}

		return stringBuilder.toString().getBytes();
	}
	
	@Override
	public void start() {
		runThread = true;
		ConsoleReporter.report("Dashboard Reporter Started!", MessageLevel.INFO);
		super.start();
	}

	public static void addDiagnosticMessage(String diagnosticMessage) {
		diagnosticMessages.add(new DiagnosticMessage(diagnosticMessage));
	}

	public static void addDiagnosticMessage(DiagnosticMessage diagnosticMessage) {
		_reporterMutex.lock();
		try {
			diagnosticMessages.add(diagnosticMessage);
		} finally {
			_reporterMutex.unlock();
		}
	}
	
}

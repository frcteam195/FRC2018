package org.usfirst.frc.team195.robot.Subsystems;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.List;

import org.usfirst.frc.team195.robot.Subsystems.*;
import org.usfirst.frc.team195.robot.Utilities.CustomSubsystem;
import org.usfirst.frc.team195.robot.Utilities.Reportable;

import edu.wpi.first.wpilibj.Timer;

public class DashboardReporter extends Thread {

	private static final int MIN_DASHBOARD_SEND_RATE_MS = 250;
	
	private static final int SEND_PORT = 5801;
	
	private static final String SEND_IP = "10.1.95.14";
	
	private InetAddress IPAddress;

    private DatagramSocket clientSocket;
    private byte[] sendData;
    private DatagramPacket sendPacket;
    
    private List<CustomSubsystem> subsystemList;
	
	private boolean runThread;
	
	private double dashboardSendThreadControlStart;
	private double dashboardSendThreadControlEnd;
	private int dashboardSendThreadControlElapsedTimeMS;
	
	private static DashboardReporter instance;
	
	public static DashboardReporter getInstance(List<CustomSubsystem> subsystemList) {
		if(instance == null) {
			try {
				instance = new DashboardReporter(subsystemList);
			} catch (Exception ex) {
				ex.printStackTrace();
			}
		}
		
		return instance;
	}
	
	private DashboardReporter(List<CustomSubsystem> subsystemList) throws Exception {
		super();
		runThread = false;
        clientSocket = new DatagramSocket(SEND_PORT);
        
	    	dashboardSendThreadControlStart = 0;
	    	dashboardSendThreadControlEnd = 0;
	    	dashboardSendThreadControlElapsedTimeMS = 0;
        
	    	IPAddress = InetAddress.getByName(SEND_IP);

        
        this.subsystemList = subsystemList;
	}
	
	@Override
	public void run() {
		while (runThread) {
			dashboardSendThreadControlStart = Timer.getFPGATimestamp();
            try {
                sendData = createSendData();
                sendPacket = new DatagramPacket(sendData, sendData.length, IPAddress, SEND_PORT);
                clientSocket.send(sendPacket);
            } catch (Exception ex) {
	        		
	        }
			do {
				dashboardSendThreadControlEnd = Timer.getFPGATimestamp();
				dashboardSendThreadControlElapsedTimeMS = (int) ((dashboardSendThreadControlEnd - dashboardSendThreadControlStart) * 1000);
				if (dashboardSendThreadControlElapsedTimeMS < MIN_DASHBOARD_SEND_RATE_MS)
					try{Thread.sleep(MIN_DASHBOARD_SEND_RATE_MS - dashboardSendThreadControlElapsedTimeMS);}catch(Exception ex) {};
			} while(dashboardSendThreadControlElapsedTimeMS < MIN_DASHBOARD_SEND_RATE_MS);
        } 
		
		if(!clientSocket.isClosed())
			clientSocket.close();
	}
	
	private byte[] createSendData() {
		String sendStr = "";
		for (CustomSubsystem customSubsystem : subsystemList) {
			if (customSubsystem instanceof Reportable)
				sendStr += ((Reportable) customSubsystem).generateReport();
		}
		return sendStr.getBytes();
	}
	
	@Override
	public void start() {
		runThread = true;
		super.start();
	}
	
}

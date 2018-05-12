package com.team195.subsystems;

import java.net.DatagramSocket;
import java.net.DatagramPacket;
import java.net.InetSocketAddress;

import com.team195.RobotMap;
import com.team195.utils.MovingAverage;
import com.team195.utils.VisionData;

public class VisionReceiver extends Thread {
	private static VisionReceiver instance = null;

	DatagramSocket server;
	byte[] clientData;
	DatagramPacket clientPacket;
	String inFromClient;

	private MovingAverage filteredDistance = new MovingAverage(6);
	private MovingAverage filteredAngle = new MovingAverage(1);

	public VisionReceiver() {
		try {
			server = new DatagramSocket(RobotMap.UDP_PORT);
			clientData = new byte[RobotMap.PACKET_LENGTH];
			clientPacket = new DatagramPacket(clientData, RobotMap.PACKET_LENGTH, new InetSocketAddress(RobotMap.UDP_IP, RobotMap.UDP_PORT));
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public static VisionReceiver getInstance() {
		if(instance == null)
			instance = new VisionReceiver();

		return instance;
	}

	public synchronized void receivePacket() {
		try {
			clientData = new byte[RobotMap.PACKET_LENGTH];
			server.receive(clientPacket);
			inFromClient = new String(clientPacket.getData()).trim();
			processPacket();
		} catch(Exception e) {
			e.printStackTrace();
    }
	}

	public synchronized void processPacket() {
		//System.out.println(inFromClient);
		while(inFromClient.indexOf(":") != -1 && inFromClient.indexOf(";") != -1) {
			String title = inFromClient.substring(0, inFromClient.indexOf(":")).toLowerCase();
			inFromClient = inFromClient.substring(inFromClient.indexOf(":") + 1);
			String value = inFromClient.substring(0, inFromClient.indexOf(";"));
			inFromClient = inFromClient.substring(inFromClient.indexOf(";") + 1);

			switch (title) {
				case "deviation":
					VisionData.rl.lock();
					filteredAngle.addNumber(Double.parseDouble(value));
					VisionData.deviation = filteredAngle.getAverage();
					VisionData.rl.unlock();
					break;
				case "distance":
					VisionData.rl.lock();
					filteredDistance.addNumber(Double.parseDouble(value)/1.8);
					VisionData.distance = filteredDistance.getAverage();
					VisionData.rl.unlock();
					break;
				case "targetfound":
					VisionData.rl.lock();
					VisionData.targetFound = Boolean.parseBoolean(value);
					VisionData.rl.unlock();
					break;
				default:
					break;
			}
		}

		//System.out.println(VisionData.printMe());
	}

	@Override
	public void run() {
		while(true) {
			receivePacket();
		}
	}
}

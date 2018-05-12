package org.usfirst.frc.team195.robot.Reporters;

import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team195.robot.AutoSelectionReceiver;
import org.usfirst.frc.team195.robot.Utilities.*;
import org.usfirst.frc.team195.robot.Utilities.Loops.RobotStateEstimator;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathFollowerRobotState;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.RigidTransform2d;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Translation2d;

import java.io.IOException;
import java.net.*;

public class MobileDiagnosticsReporter {
	public static final int MIN_AUTO_RECEIVE_RATE_MS = 250;
	private static MobileDiagnosticsReporter instance = null;

	private boolean runThread;
	private ThreadRateControl threadRateControl = new ThreadRateControl();
	private int portNumber = 5807;	//Default port number

	private byte[] sendData;
	private DatagramPacket sendPacket;
	private byte[] receiveData;
	private DatagramPacket receivePacket;
	private DatagramSocket clientSocket;
	private Thread t;

	private InetAddress IPAddress;

	private MobileDiagnosticsReporter() throws Exception {
		super();
		runThread = false;
	}

	public static MobileDiagnosticsReporter getInstance() {
		if(instance == null) {
			try {
				instance = new MobileDiagnosticsReporter();
			} catch (Exception ex) {
				ConsoleReporter.report(ex, MessageLevel.DEFCON1);
			}
		}

		return instance;
	}

	public synchronized void setPortNumber(int portNumber) {
		this.portNumber = portNumber;
	}

	public synchronized void start() {
		if (t == null || !t.isAlive()) {
			runThread = true;

			t = new Thread(() -> {
				t.setPriority(Thread.NORM_PRIORITY);
				try {
					clientSocket = new DatagramSocket(this.portNumber);
				} catch (Exception ex) {
					ConsoleReporter.report(ex, MessageLevel.ERROR);
				}

				threadRateControl.start();
				while (runThread) {
					receiveData = new byte[1024];
					receivePacket = new DatagramPacket(receiveData, receiveData.length, new InetSocketAddress(0).getAddress(), portNumber);
					try {
						clientSocket.receive(receivePacket);
						setIPAddress(receivePacket.getAddress());
						processUDPPacket(receivePacket.getData());
					} catch (IOException e) {
						e.printStackTrace();
					}

					threadRateControl.doRateControl(MIN_AUTO_RECEIVE_RATE_MS);
				}

				clientSocket.close();

			});

			t.start();
		}
	}

	private synchronized void setIPAddress(InetAddress ipAddr) {
		try {
			IPAddress = ipAddr;
		} catch (Exception ex) {
			IPAddress = null;
		}
	}

	public synchronized void terminate() {
		runThread = false;
	}

	private void processUDPPacket(byte[] data) {
		String sData = (new String(data)).trim();
		String[] sArr = sData.split(";");
		for (String s : sArr) {
			String[] sArrProcess = s.split(":");
			if (sArrProcess.length == 2) {
				String command = sArrProcess[0].toLowerCase();
				String value = sArrProcess[1].toLowerCase();

				if (command.equals("getdata")) {
					try {
						switch (value) {
							case "diagnostics":
								sendData = DashboardReporter.getInstance(null).getSendData().clone();
								break;
							case "autonomous":
								sendData = buildAutoTrackerData();
								break;
							default:
								sendData = null;
								break;
						}

						if (IPAddress != null && sendData != null) {
							sendPacket = new DatagramPacket(sendData, sendData.length, IPAddress, portNumber);
							try {
								clientSocket.send(sendPacket);
							} catch (IOException e) {
								e.printStackTrace();
							}
						}

					} catch (Exception ex) {
						if (Constants.DEBUG)
							ConsoleReporter.report(ex.toString(), MessageLevel.ERROR);
					}
				}
			}
		}
	}

	private byte[] buildAutoTrackerData() {
		//RigidTransform2d tmp = PathFollowerRobotState.getInstance().getLatestFieldToVehicle().getValue();
		RigidTransform2d tmp = new RigidTransform2d();
		String retVal = "";
		retVal += "Alliance:" + DriverStation.getInstance().getAlliance() + ";";
		retVal += "RobotX:" + tmp.getTranslation().x() + ";";
		retVal += "RobotY:" + tmp.getTranslation().y() + ";";
		retVal += "RobotAngle:" + tmp.getRotation().getDegrees() + ";";
		retVal += "StartPos:" + AutoSelectionReceiver.getInstance().getStartingPosition() + ";";
		return retVal.getBytes();
	}
}

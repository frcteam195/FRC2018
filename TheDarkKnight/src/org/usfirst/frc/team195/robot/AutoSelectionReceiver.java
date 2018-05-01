package org.usfirst.frc.team195.robot;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.Constants;
import org.usfirst.frc.team195.robot.Utilities.ScaleHeightPriority;
import org.usfirst.frc.team195.robot.Utilities.StartingPosition;
import org.usfirst.frc.team195.robot.Utilities.ThreadRateControl;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;

public class AutoSelectionReceiver {
	public static final int MIN_AUTO_RECEIVE_RATE_MS = 250;
	private static AutoSelectionReceiver instance = null;

	private boolean runThread;
	private ThreadRateControl threadRateControl = new ThreadRateControl();
	private int portNumber = 5803;	//Default port number

	private byte[] receiveData;
	private DatagramPacket receivePacket;
	private DatagramSocket clientSocket;
	private Thread t;

	private StartingPosition mTempStartingPosition = StartingPosition.RIGHT;
	private ScaleHeightPriority mTempScalePriority = ScaleHeightPriority.LOW;


	private StartingPosition startingPosition = StartingPosition.RIGHT;
	private ScaleHeightPriority scaleHeightPriority = ScaleHeightPriority.LOW;

	private AutoSelectionReceiver() throws Exception {
		super();
		runThread = false;
	}

	public static AutoSelectionReceiver getInstance() {
		if(instance == null) {
			try {
				instance = new AutoSelectionReceiver();
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
						processUDPPacket(receivePacket.getData());
						startingPosition = mTempStartingPosition;
						scaleHeightPriority = mTempScalePriority;
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

	public synchronized void terminate() {
		runThread = false;
	}

	public StartingPosition getStartingPosition() {
		return startingPosition;
	}

	public ScaleHeightPriority getScaleHeightPriority() {
		return scaleHeightPriority;
	}

	private void processUDPPacket(byte[] data) {
		String sData = (new String(data)).trim();
		String[] sArr = sData.split(";");
		int autoStartPos = -1;
		int scaleHeight = -1;
		for (String s : sArr) {
			String[] sArrProcess = s.split(":");
			if (sArrProcess.length == 2) {
				String command = sArrProcess[0].toLowerCase();
				String value = sArrProcess[1];

				try {
					switch (command) {
						case "autonomous":
							autoStartPos = Integer.parseInt(value);
							break;
						case "scale":
							scaleHeight = Integer.parseInt(value);
							break;
						default:
							break;
					}
				} catch (Exception ex) {
					autoStartPos = -1;
					scaleHeight = -1;

					if (Constants.DEBUG)
						ConsoleReporter.report(ex.toString(), MessageLevel.ERROR);
				}
			}
		}

		switch (autoStartPos) {
			case 0:
				mTempStartingPosition = StartingPosition.RIGHT;
				break;
			case 1:
				mTempStartingPosition = StartingPosition.CENTER;
				break;
			case 2:
				mTempStartingPosition = StartingPosition.LEFT;
				break;
			default:
				mTempStartingPosition = StartingPosition.RIGHT;
				break;
		}

		switch (autoStartPos) {
			case 0:
				mTempScalePriority = ScaleHeightPriority.LOW;
				break;
			case 1:
				mTempScalePriority = ScaleHeightPriority.HIGH;
				break;
			default:
				mTempScalePriority = ScaleHeightPriority.LOW;
				break;
		}
	}

}

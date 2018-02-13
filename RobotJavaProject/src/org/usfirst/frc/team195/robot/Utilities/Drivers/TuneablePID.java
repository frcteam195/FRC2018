package org.usfirst.frc.team195.robot.Utilities.Drivers;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.Constants;
import org.usfirst.frc.team195.robot.Utilities.SetpointValue;
import org.usfirst.frc.team195.robot.Utilities.ThreadRateControl;

public class TuneablePID {
	public static final int MIN_PID_SEND_RATE_MS = 10;
	public static final int MIN_PID_RECEIVE_RATE_MS = 40;

	protected UDPReceiver udpReceiver;
	protected UDPSender udpSender;
	
	private String name;
	private ArrayList<TalonSRX> talonArrList;
	private SetpointValue setpointReq;
	private int portNumber;
	private boolean autoUpdate;
	private boolean autoUpdateSetpoint;

	private InetAddress IPAddress;
	private DatagramSocket clientSocket;
	private byte[] sendData;
	private DatagramPacket sendPacket;
	private byte[] receiveData;
	private DatagramPacket receivePacket;

	public TuneablePID(String name, TalonSRX tuningTalon, SetpointValue setpointReq, int portNumber, boolean autoUpdate,
					   boolean autoUpdateSetpoint) throws Exception {
		this(name, new ArrayList<TalonSRX>(Arrays.asList(tuningTalon)), setpointReq, portNumber, autoUpdate,
				autoUpdateSetpoint);
	}

	public TuneablePID(String name, TalonSRX tuningTalon, TalonSRX tuningTalon2, SetpointValue setpointReq, int portNumber,
			boolean autoUpdate, boolean autoUpdateSetpoint) throws Exception {
		this(name, new ArrayList<TalonSRX>(Arrays.asList(tuningTalon, tuningTalon2)), setpointReq, portNumber,
				autoUpdate, autoUpdateSetpoint);
	}

	public TuneablePID(String name, ArrayList<TalonSRX> talonArrList, SetpointValue setpointReq, int portNumber,
			boolean autoUpdate, boolean autoUpdateSetpoint) throws Exception {
		this.name = name;
		this.talonArrList = talonArrList;
		this.setpointReq = setpointReq;
		this.portNumber = portNumber;
		this.autoUpdate = autoUpdate;
		this.autoUpdateSetpoint = autoUpdateSetpoint;

		clientSocket = new DatagramSocket(this.portNumber);
		IPAddress = null;
		udpReceiver = new UDPReceiver();
		udpSender = new UDPSender();

	}
	
	public void start() {
		if (Constants.TUNING_PIDS && !DriverStation.getInstance().isFMSAttached()) {
			if (udpReceiver != null && udpSender != null) {
				udpReceiver.start();
				udpSender.start();
				ConsoleReporter.report("Starting PID Tuner for " + name, MessageLevel.INFO);
			}
		}
	}

	public boolean isAlive() {
		return udpReceiver.isAlive() || udpSender.isAlive();
	}

	public void terminate() {
		if (udpReceiver != null && udpSender != null) {
			udpReceiver.terminate();
			udpSender.terminate();
		}
	}

	protected InetAddress getIPAddress() {
		return IPAddress;
	}

	protected synchronized void setIPAddress(InetAddress ipAddr) {
		try {
			IPAddress = ipAddr;
		} catch (Exception ex) {
			IPAddress = null;
		}
	}

	private class UDPSender extends Thread {
		private boolean runThread;
		private ThreadRateControl threadRateControl = new ThreadRateControl();

		public UDPSender() throws Exception {
			super();
			runThread = false;
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
			threadRateControl.start();
			while (runThread) {

				if (getIPAddress() != null) {
					sendData = createSendData();
					sendPacket = new DatagramPacket(sendData, sendData.length, getIPAddress(), portNumber);
					try {
						clientSocket.send(sendPacket);
					} catch (IOException e) {
						e.printStackTrace();
					}
				}

				threadRateControl.doRateControl(MIN_PID_SEND_RATE_MS);
			}
		}

		private byte[] createSendData() {
			double setpoint = setpointReq != null ? setpointReq.value : 0;
			double actualValue = 0;
			int sensorSelect = 0;
			if (talonArrList.size() > sensorSelect) {
				switch(talonArrList.get(sensorSelect).getControlMode()) {
					case Position:
						actualValue = talonArrList.get(sensorSelect).getSelectedSensorPosition(0) / Constants.kSensorUnitsPerRotation;
						break;
					case Current:
						actualValue = talonArrList.get(sensorSelect).getOutputCurrent();
						break;
					case Velocity:
						actualValue = talonArrList.get(sensorSelect).getSelectedSensorVelocity(0) / Constants.kSensorUnitsPerRotation * 600;
						break;
					case MotionMagic:
						actualValue = talonArrList.get(sensorSelect).getSelectedSensorPosition(0) / Constants.kSensorUnitsPerRotation;
						break;
					default:
						//actualValue = talonArrList.get(sensorSelect).getSelectedSensorVelocity(0) / Constants.kSensorUnitsPerRotation * 600;
						actualValue = talonArrList.get(sensorSelect).getOutputCurrent();
						break;
				}
			}

			double iAccum = talonArrList.get(sensorSelect).getIntegralAccumulator(0);

			if (autoUpdateSetpoint)
				setpoint = udpReceiver.getTuneablePIDData().getSetpoint();

			String sendStr = "Name:" + name + ";DesiredValue:" + setpoint + ";ActualValue:" + actualValue + ";IntegralAccum:" + iAccum + ";";
			return sendStr.getBytes();
		}
	}

	private class UDPReceiver extends Thread {
		private boolean runThread;
		private TuneablePIDData tuneablePIDData;
		private ThreadRateControl threadRateControl = new ThreadRateControl();

		public UDPReceiver() {
			super();
			runThread = false;
			tuneablePIDData = new TuneablePIDData(0, 0, 0, 0, 0, 0, 0);
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
			threadRateControl.start();
			while (runThread) {
				receiveData = new byte[1024];
				receivePacket = new DatagramPacket(receiveData, receiveData.length, new InetSocketAddress(0).getAddress(), portNumber);
				try {
					clientSocket.receive(receivePacket);
					setIPAddress(receivePacket.getAddress());
					setTuneablePIDData(processUDPPacket(receivePacket.getData()));
					for (TalonSRX tuningTalon : talonArrList) {
						if (autoUpdate) {
							TalonHelper.setPIDGains(tuningTalon, 0, tuneablePIDData.getkP(), tuneablePIDData.getkI(), tuneablePIDData.getkD(), tuneablePIDData.getF(), tuneablePIDData.getRampRate(), (int)tuneablePIDData.getiZone());

							if (tuningTalon.getControlMode() == ControlMode.MotionMagic)
								TalonHelper.setMotionMagicParams(tuningTalon, (int)tuneablePIDData.getCruiseVelocity(), (int)tuneablePIDData.getAccel());

							if (autoUpdateSetpoint)
								switch(tuningTalon.getControlMode()) {
									case MotionMagic:
										tuningTalon.set(tuningTalon.getControlMode(), tuneablePIDData.getSetpoint() * Constants.kSensorUnitsPerRotation);
										break;
									case Velocity:
										tuningTalon.set(tuningTalon.getControlMode(), tuneablePIDData.getSetpoint() * Constants.kSensorUnitsPerRotation / 600);
										break;
									default:
										tuningTalon.set(tuningTalon.getControlMode(), tuneablePIDData.getSetpoint());
										break;
								}
						}
					}
				} catch (IOException e) {
					e.printStackTrace();
				}

				threadRateControl.doRateControl(MIN_PID_RECEIVE_RATE_MS);
			}
		}
		
		public TuneablePIDData getTuneablePIDData() {
			return tuneablePIDData;
		}

		private synchronized void setTuneablePIDData(TuneablePIDData t) {
			tuneablePIDData = t;
		}

		protected TuneablePIDData processUDPPacket(byte[] data) {
			String sData = (new String(data)).trim();
			String[] sArr = sData.split(";");
			double kP = -1;
			double kI = -1;
			double kD = -1;
			double f = -1;
			double rampRate = -1;
			double iZone = -1;
			double setpoint = -1;
			double cruiseVelocity = -1;
			double accel = -1;
			for (String s : sArr) {
				String[] sArrProcess = s.split(":");
				if (sArrProcess.length == 2) {
					String command = sArrProcess[0].toLowerCase();
					String value = sArrProcess[1];

					try {
						switch (command) {
						case "kp":
							kP = Double.parseDouble(value);
							break;
						case "ki":
							kI = Double.parseDouble(value);
							break;
						case "kd":
							kD = Double.parseDouble(value);
							break;
						case "f":
							f = Double.parseDouble(value);
							break;
						case "ramprate":
							rampRate = Double.parseDouble(value);
							break;
						case "izone":
							iZone = Double.parseDouble(value);
							break;
						case "setpoint":
							setpoint = Double.parseDouble(value);
							break;
						case "cruisevelocity":
							cruiseVelocity = Double.parseDouble(value);
							break;
						case "acceleration":
							accel = Double.parseDouble(value);
							break;
						default:
							break;
						}
					} catch (Exception ex) {
						if (Constants.DEBUG)
							ConsoleReporter.report(ex.toString(), MessageLevel.ERROR);
					}
				}
			}

			return new TuneablePIDData(kP, kI, kD, f, setpoint, cruiseVelocity, accel, rampRate, iZone);
		}
	}

	private class TuneablePIDData {
		private double kP;
		private double kI;
		private double kD;
		private double f;
		private double setpoint;
		private double rampRate;
		private double iZone;
		private double cruiseVelocity;
		private double accel;
		
		public TuneablePIDData(double kP, double kI, double kD, double f, double setpoint, double cruiseVelocity, double accel) {
			this.kP = kP;
			this.kI = kI;
			this.kD = kD;
			this.f = f;
			this.setpoint = setpoint;
			this.cruiseVelocity = cruiseVelocity;
			this.accel = accel;
			rampRate = 0;
			iZone = 0;
		}

		public TuneablePIDData(double kP, double kI, double kD, double f, double setpoint, double cruiseVelocity, double accel, double rampRate, double iZone) {
			this.kP = kP;
			this.kI = kI;
			this.kD = kD;
			this.f = f;
			this.setpoint = setpoint;
			this.cruiseVelocity = cruiseVelocity;
			this.accel = accel;
			this.rampRate = rampRate;
			this.iZone = iZone;
		}
		
		public double getkP() {
			return kP;
		}
		public double getkI() {
			return kI;
		}
		public double getkD() {
			return kD;
		}
		public double getF() {
			return f;
		}
		public double getSetpoint() {
			return setpoint;
		}
		public double getRampRate() { return rampRate; }
		public double getiZone() { return iZone; }
		public double getCruiseVelocity() {
			return cruiseVelocity;
		}
		public double getAccel() {
			return accel;
		}
	}
}

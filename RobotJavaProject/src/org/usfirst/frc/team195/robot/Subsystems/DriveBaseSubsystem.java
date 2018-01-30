package org.usfirst.frc.team195.robot.Subsystems;

import java.io.Console;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.Thread;

import java.util.List;
import java.util.concurrent.Semaphore;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.DashboardReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.*;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Utilities.SplineMotion.SRX.SRXDriveBaseTrajectory;

public class DriveBaseSubsystem extends Thread implements CustomSubsystem, Reportable {
	public static final int MIN_DRIVE_LOOP_TIME_STANDARD = 10;
	public static final int MIN_DRIVE_LOOP_TIME_MP = 3;

	private TuneablePID tuneableLeftDrive;
	private TuneablePID tuneableRightDrive;
	private boolean mPrevShiftVal;
	private boolean mPrevBrakeModeVal;

	@Override
	public void init() {
		leftDrive.setSensorPhase(true);

		rightDrive.setInverted(true);
		rightDriveSlave1.setInverted(true);
		rightDriveSlave2.setInverted(true);
		rightDrive.setSensorPhase(true);

		leftDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		leftDrive.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kTimeoutMs);
		leftDrive.configVelocityMeasurementWindow(32, Constants.kTimeoutMs);

		rightDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		rightDrive.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kTimeoutMs);
		rightDrive.configVelocityMeasurementWindow(32, Constants.kTimeoutMs);
		
		leftDrive.configMotionProfileTrajectoryPeriod(0, Constants.kTimeoutMs);
		leftDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
		rightDrive.configMotionProfileTrajectoryPeriod(0, Constants.kTimeoutMs);
		rightDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		boolean leftSensorPresent = leftDrive.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;
		boolean rightSensorPresent = rightDrive.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;
		if (!leftSensorPresent || !rightSensorPresent) {
			String msg = "Could not detect encoder! \r\n\tLeft Encoder Detected: " + leftSensorPresent + "\r\n\tRight Encoder Detected: " + rightSensorPresent;
			ConsoleReporter.report(msg, MessageLevel.DEFCON1);
			DriverStation.reportError(msg, false);
		}

		setLeftDrivePID(0.2, 0, 2, 0.1, 0); //setLeftDrivePID(0.4, 0, 4, 0.35, 0);
		setLeftDrivePID(0.2, 0, 2, 0.121, 1); //setLeftDrivePID(0.2, 0, 2, 0.121, 1);
		setRightDrivePID(0.2, 0, 2, 0.1, 0); //setRightDrivePID(0.4, 0, 4, 0.37, 0);
		setRightDrivePID(0.2, 0, 2, 0.134, 1); //setRightDrivePID(0.2, 0, 2, 0.134, 1);
	}
	
	@Override
	public void start() {
		runThread = true;
		tuneableLeftDrive.start();
		tuneableRightDrive.start();
		super.start();
	}

	public void terminate() {
		runThread = false;
	}
	
	@Override
	public void subsystemHome() {
		navX.zeroYaw();
		leftDrive.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
		rightDrive.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
		try {Thread.sleep(20);} catch (Exception ex) {}
	}
	
	@Override
	public void run() {
		while (!ds.isEnabled()) {try{Thread.sleep(20);}catch(Exception ex) {}}
		subsystemHome();

		while(runThread) {
			driveThreadControlStart = Timer.getFPGATimestamp();

			if (requestSetPosition) {
				leftDrive.setSelectedSensorPosition((int)position, 0, Constants.kTimeoutMs);
				rightDrive.setSelectedSensorPosition((int)position, 0, Constants.kTimeoutMs);
				try {
					_subsystemMutex.acquire();
					requestSetPosition = false;
					_subsystemMutex.release();
				} catch (Exception ex) {
					ConsoleReporter.report(ex.toString(), MessageLevel.ERROR);
				}
			}

			ControlMode ctrlMode = leftDrive.getControlMode();

			if(ctrlMode != requestedControlMode) {
				ConsoleReporter.report("Changing Control Modes!", MessageLevel.INFO);

				switch (requestedControlMode) {
					case MotionProfile:
						ConsoleReporter.report("Set to motion profile disabled!", MessageLevel.INFO);
						leftDrive.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
						rightDrive.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
						setBrakeMode(true);
						break;
					case Velocity:
						leftDrive.set(ControlMode.Velocity, 0);
						rightDrive.set(ControlMode.Velocity, 0);
						setBrakeMode(true);
						break;
					default:
						leftDrive.set(ControlMode.PercentOutput, 0);
						rightDrive.set(ControlMode.PercentOutput, 0);
						break;
				}
				ctrlMode = leftDrive.getControlMode();
			}

			switch (ctrlMode) {
				case MotionProfile:
					leftDrive.getMotionProfileStatus(mpStatusLeft);
					rightDrive.getMotionProfileStatus(mpStatusRight);

					if(mpStatusLeft.hasUnderrun){
						leftDrive.clearMotionProfileHasUnderrun(Constants.kTimeoutMs);
						ConsoleReporter.report("Left Drive Underrun", MessageLevel.ERROR);
					}
					if(mpStatusRight.hasUnderrun){
						rightDrive.clearMotionProfileHasUnderrun(Constants.kTimeoutMs);
						ConsoleReporter.report("Right Drive Underrun", MessageLevel.ERROR);
					}

					leftDrive.processMotionProfileBuffer();
					rightDrive.processMotionProfileBuffer();

					if (mpStatusLeft.activePointValid && mpStatusLeft.isLast) {
						leftDrive.set(ControlMode.MotionProfile, SetValueMotionProfile.Hold.value);
						ConsoleReporter.report("Hold Left", MessageLevel.INFO);
					}
					else if (startMPLeft) {
						try {
							_subsystemMutex.acquire();
							startMPLeft = false;
							_subsystemMutex.release();
						} catch (Exception ex) {
							ConsoleReporter.report(ex.toString(), MessageLevel.ERROR);
						}
						leftDrive.set(ControlMode.MotionProfile, SetValueMotionProfile.Enable.value);
						ConsoleReporter.report("Enabling Left SplineMotion Profile", MessageLevel.INFO);
					}
					else if (mpStatusLeft.topBufferCnt == 0 && mpStatusLeft.btmBufferCnt == 0) {
						//leftDrive.Set(ControlMode.MotionProfile, SetValueMotionProfile.Disable);
						//cout << "Bad2Left" << endl;
					}

					if (mpStatusRight.activePointValid && mpStatusRight.isLast) {
						rightDrive.set(ControlMode.MotionProfile, SetValueMotionProfile.Hold.value);
						ConsoleReporter.report("Hold Right", MessageLevel.INFO);
					}
					else if (startMPRight) {
						try {
							_subsystemMutex.acquire();
							startMPRight = false;
							_subsystemMutex.release();
						} catch (Exception ex) {
							ConsoleReporter.report(ex.toString(), MessageLevel.ERROR);
						}
						rightDrive.set(ControlMode.MotionProfile, SetValueMotionProfile.Enable.value);
						ConsoleReporter.report("Enabling Right SplineMotion Profile", MessageLevel.INFO);
					}
					else if (mpStatusRight.topBufferCnt == 0 && mpStatusRight.btmBufferCnt == 0) {
						//rightDrive.Set(ControlMode.MotionProfile, SetValueMotionProfile.Disable);
						//cout << "Bad2Right" << endl;
					}
					//ConsoleReporter.report("SplineMotion Loop!", MessageLevel.INFO);
					break;
				case Velocity:
					//leftDrive.set(ControlMode.Velocity, leftDriveSpeed * Constants.kSensorUnitsPerRotation / 600);
					//rightDrive.set(ControlMode.Velocity, rightDriveSpeed * Constants.kSensorUnitsPerRotation / 600);
					break;
				case PercentOutput:
				default:
					//leftDrive.set(ControlMode.PercentOutput, leftDriveSpeed);
					//rightDrive.set(ControlMode.PercentOutput, rightDriveSpeed);
					break;
			}

			if (holdLow) {
				shift(false);
			} else if (highGear) {
				shift(true);
			} else {
				shift(false);
			}

			int loopRate = (ctrlMode == ControlMode.MotionProfile ? MIN_DRIVE_LOOP_TIME_MP : MIN_DRIVE_LOOP_TIME_STANDARD);

			do {
				driveThreadControlEnd = Timer.getFPGATimestamp();
				driveThreadControlElapsedTimeMS = (int) ((driveThreadControlEnd - driveThreadControlStart) * 1000);
				if (driveThreadControlElapsedTimeMS < loopRate)
					try{Thread.sleep(loopRate - driveThreadControlElapsedTimeMS);}catch(Exception ex) {};
			} while(driveThreadControlElapsedTimeMS < loopRate);
		}
	}

	public double getAveragePosition() {
		return (Math.abs(leftDrive.getSelectedSensorPosition(0)) + Math.abs(rightDrive.getSelectedSensorPosition(0))) / 2;
	}
	public double getLeftDrivePosition() {
		return leftDrive.getSelectedSensorPosition(0);
	}
	public double getRightDrivePosition() {
		return rightDrive.getSelectedSensorPosition(0);
	}

	public synchronized void setDriveSpeed(DriveMotorValues d) {
		setDriveSpeed(d.leftDrive, d.rightDrive, false);
	}
	public synchronized void setDriveSpeed(double leftDriveSpeed, double rightDriveSpeed) {
		setDriveSpeed(leftDriveSpeed, rightDriveSpeed, false);
	}

	private void shift(boolean highGear) {
		try {
			_subsystemMutex.acquire();
			if (mPrevShiftVal != highGear) {
				if (highGear) {
					ConsoleReporter.report("Setting drive gains for high gear!", MessageLevel.INFO);
					leftDrive.selectProfileSlot(0, 1);
					rightDrive.selectProfileSlot(0, 1);
				} else {
					ConsoleReporter.report("Setting drive gains for low gear!", MessageLevel.INFO);
					leftDrive.selectProfileSlot(0, 0);
					rightDrive.selectProfileSlot(0, 0);
				}

				shiftSol.set(highGear);
				mPrevShiftVal = highGear;
			}
			_subsystemMutex.release();
		} catch (Exception ex) {
			StringWriter s = new StringWriter();
			ex.printStackTrace(new PrintWriter(s));
			ConsoleReporter.report(s.toString(), MessageLevel.ERROR);
		}
	}
	
	public synchronized void setDriveSpeed(double leftDriveSpeed, double rightDriveSpeed, boolean slowDown) {
		try {
			_subsystemMutex.acquire();
			if(slowDown) {
				this.leftDriveSpeed = leftDriveSpeed / 2.2;
				this.rightDriveSpeed = rightDriveSpeed / 2.2;
			}
			else {
				this.leftDriveSpeed = leftDriveSpeed;
				this.rightDriveSpeed = rightDriveSpeed;
			}
			_subsystemMutex.release();
		} catch (Exception ex) {
			StringWriter s = new StringWriter();
			ex.printStackTrace(new PrintWriter(s));
			ConsoleReporter.report(s.toString(), MessageLevel.ERROR);
		}
	}
	
	public synchronized void setHoldLowGear(boolean holdLowGear) {
		this.holdLow = holdLowGear;
	}
	
	public synchronized void setGear(boolean highGear) {
		this.highGear = highGear;
	}
	
	public synchronized void setPosition(double position) {
		try {
			_subsystemMutex.acquire();
			requestSetPosition = true;
			this.position = position;
			_subsystemMutex.release();
		} catch (Exception ex) {
			ConsoleReporter.report(ex.toString(), MessageLevel.ERROR);
		}
	}
	
	public void startMPTrajectory() {
		try {
			ConsoleReporter.report("Starting SplineMotion!", MessageLevel.INFO);
			_subsystemMutex.acquire();
			startMPLeft = true;
			startMPRight = true;
			_subsystemMutex.release();
		} catch (Exception ex) {
			ConsoleReporter.report(ex.toString(), MessageLevel.ERROR);
		}
	}
	
	public boolean isHighGear() {
		return highGear;
	}

	public MotionProfileStatus getLeftMPStatus() {
		return mpStatusLeft;
	}
	public MotionProfileStatus getRightMPStatus() {
		return mpStatusRight;
	}

	public synchronized void setMotionProfileTrajectory(SRXDriveBaseTrajectory srxDriveBaseTrajectory) {
		setMotionProfileTrajectory(srxDriveBaseTrajectory.getLeftWheelTrajectory(), srxDriveBaseTrajectory.getRightWheelTrajectory());
	}

	public synchronized void setMotionProfileTrajectory(double[][] mpLeftBuffer, double[][] mpRightBuffer) {
		this.mpLeftBuffer = mpLeftBuffer;
		this.mpRightBuffer = mpRightBuffer;
		Thread leftMPBufferThread = new Thread(() -> {
			processMPLeft();
		});
		Thread rightMPBufferThread = new Thread(() -> {
			processMPRight();
		});
		leftMPBufferThread.start();
		rightMPBufferThread.start();
		try {
			leftMPBufferThread.join();
			rightMPBufferThread.join();
		} catch (Exception ex) {
			ConsoleReporter.report(ex.toString(), MessageLevel.ERROR);
		}
	}
	
	public void setControlMode(ControlMode controlMode) {
		if (controlMode != requestedControlMode) {
			try {
				_subsystemMutex.acquire();
				requestedControlMode = controlMode;
				_subsystemMutex.release();
			} catch (Exception ex) {
				ConsoleReporter.report(ex.toString(), MessageLevel.ERROR);
			}
		}
	}
	
	public ControlMode getControlMode() {
		return requestedControlMode;
	}

	public synchronized void setDrivePID(double kP, double kI, double kD, double ff, int profileNum) {
		setLeftDrivePID(kP, kI, kD, ff, profileNum);
		setRightDrivePID(kP, kI, kD, ff, profileNum);
	}
	
	public synchronized void setLeftDrivePID(double kP, double kI, double kD, double ff, int profileNum) {
		TalonHelper.setPIDGains(leftDrive, profileNum, kP, kI, kD, ff);
	}
	
	public synchronized void setRightDrivePID(double kP, double kI, double kD, double ff, int profileNum) {
		TalonHelper.setPIDGains(rightDrive, profileNum, kP, kI, kD, ff);
	}
	public boolean isPositionWithinRange(double range) {
		return ((Math.abs(leftDrive.getSelectedSensorPosition(0)) - Math.abs(leftDriveSpeed)) < range && (Math.abs(rightDrive.getSelectedSensorPosition(0)) - Math.abs(rightDriveSpeed)) < range);
	}

	public synchronized void setMotionMagicVelocityAccel(double vel, double accel) {
		leftDrive.configMotionCruiseVelocity((int)vel, Constants.kTimeoutMs);
		leftDrive.configMotionAcceleration((int)accel, Constants.kTimeoutMs);
		rightDrive.configMotionCruiseVelocity((int)vel, Constants.kTimeoutMs);
		rightDrive.configMotionAcceleration((int)accel, Constants.kTimeoutMs);	
	}

	public static DriveBaseSubsystem getInstance() {
		if(instance == null) {
			try {
				instance = new DriveBaseSubsystem();
			} catch (Exception ex) {
				ConsoleReporter.report(ex.toString(), MessageLevel.DEFCON1);
			}
		}
		
		return instance;
	}
	
	public static DriveBaseSubsystem getInstance(List<CustomSubsystem> subsystemList) {
		subsystemList.add(getInstance());
		return instance;
	}

	public synchronized void setBrakeMode(boolean brakeMode) {
		if (mPrevBrakeModeVal != brakeMode) {
			leftDrive.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			leftDriveSlave1.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			leftDriveSlave2.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			rightDrive.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			rightDriveSlave1.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			rightDriveSlave2.setNeutralMode(brakeMode ? NeutralMode.Brake : NeutralMode.Coast);
			mPrevBrakeModeVal = brakeMode;
		}
	}
	
	@Override
	public String toString() {
		return generateReport();
	}

	private DriveBaseSubsystem() throws Exception {
		super();
		ds = DriverStation.getInstance();
		_subsystemMutex = new Semaphore(1);

		Controllers robotControllers = Controllers.getInstance();
		leftDrive = robotControllers.getLeftDrive1();
		leftDriveSlave1 = robotControllers.getLeftDrive2();
		leftDriveSlave2 = robotControllers.getLeftDrive3();
		rightDrive = robotControllers.getRightDrive1();
		rightDriveSlave1 = robotControllers.getRightDrive2();
		rightDriveSlave2 = robotControllers.getRightDrive3();

		shiftSol = robotControllers.getShiftSol();

		navX = robotControllers.getNavX();


		highGear = false;
		mPrevShiftVal = false;
		shift(false);

		mPrevBrakeModeVal = false;
		setBrakeMode(true);

		holdLow = false;
		leftDriveSpeed = 0;
		rightDriveSpeed = 0;

		runThread = false;

		driveThreadControlStart = 0;
		driveThreadControlEnd = 0;
		driveThreadControlElapsedTimeMS = 0;

		mpLeftBuffer = null;
		mpRightBuffer = null;

		requestSetPosition = false;
		position = 0;

		requestedControlMode = ControlMode.MotionProfile;
		startMPLeft = false;
		startMPRight = false;

		mpStatusLeft = new MotionProfileStatus();
		mpStatusRight = new MotionProfileStatus();

		tuneableLeftDrive = new TuneablePID("LeftDrive", leftDrive, 0, 5808, true, true);
		tuneableRightDrive = new TuneablePID("RightDrive", rightDrive, 0, 5809, true, true);
	}

	private static DriveBaseSubsystem instance = null;

	private ControlMode requestedControlMode;

	private boolean startMPLeft;
	private boolean startMPRight;

	private void processMPLeft() {
		processMP(leftDrive, mpLeftBuffer);
	}
	private void processMPRight() {
		processMP(rightDrive, mpRightBuffer);
	}
	private void processMP(TalonSRX talonSRX, double[][] mpBuffer) {
		talonSRX.clearMotionProfileTrajectories();

		for (int i = 0; i < mpBuffer.length; i++) {
			TrajectoryPoint point = new TrajectoryPoint();
			try {
				point.position = mpBuffer[i][0] * Constants.kSensorUnitsPerRotation;
				point.velocity = mpBuffer[i][1] * Constants.kSensorUnitsPerRotation / 600;
				point.headingDeg = 0;
				point.timeDur = GetTrajectoryDuration((int)mpBuffer[i][2]);
				point.profileSlotSelect0 = 0;
				point.profileSlotSelect1 = 0;
			} catch (Exception ex) {
				ConsoleReporter.report(ex.toString(), MessageLevel.ERROR);
			}

			/*
			if (highGear)
				point.profileSlotSelect0 = 1;
			else
				point.profileSlotSelect0 = 0;
			 */

			//Set at start
			point.zeroPos = i == 0;

			//Set at end
			point.isLastPoint = (i + 1) == mpBuffer.length;

			talonSRX.pushMotionProfileTrajectory(point);
		}
	}

	private TrajectoryDuration GetTrajectoryDuration(int durationMs)
	{	 
		TrajectoryDuration retval = TrajectoryDuration.Trajectory_Duration_0ms;
		retval = retval.valueOf(durationMs);
		if (retval.value != durationMs) {
			ConsoleReporter.report("SplineMotion Duration not supported - use configMotionProfileTrajectoryPeriod instead", MessageLevel.ERROR);
		}
		return retval;
	}

	private double driveThreadControlStart, driveThreadControlEnd;
	private int driveThreadControlElapsedTimeMS;

	private DriverStation ds;

	private double leftDriveSpeed;
	private double rightDriveSpeed;
	private boolean highGear, holdLow;
	private double[][] mpLeftBuffer;
	private double[][] mpRightBuffer;

	private boolean requestSetPosition;
	private double position;

	private MotionProfileStatus mpStatusLeft;
	private MotionProfileStatus mpStatusRight;
	private TalonSRX leftDrive;
	private VictorSPX leftDriveSlave1;
	private VictorSPX leftDriveSlave2;
	private TalonSRX rightDrive;
	private VictorSPX rightDriveSlave1;
	private VictorSPX rightDriveSlave2;

	private AHRS navX;

	private Solenoid shiftSol;

	private boolean runThread;
	
	protected Semaphore _subsystemMutex;

	@Override
	public String generateReport() {
		String retVal = "";
		
		retVal += "LeftDrivePos:" + leftDrive.getSelectedSensorVelocity(0) + ";";
		retVal += "LeftDriveVel:" + leftDrive.getSelectedSensorPosition(0) + ";";
		retVal += "LeftDriveOutput:" + leftDriveSpeed + ";";
		retVal += "LeftDrive1Current:" + leftDrive.getOutputCurrent() + ";";
		retVal += "LeftDrive2Current:" + leftDriveSlave1.getOutputCurrent() + ";";
		retVal += "LeftDrive3Current:" + leftDriveSlave2.getOutputCurrent() + ";";
		
		retVal += "RightDrivePos:" + rightDrive.getSelectedSensorVelocity(0) + ";";
		retVal += "RightDriveVel:" + rightDrive.getSelectedSensorPosition(0) + ";";
		retVal += "RightDriveOutput:" + rightDriveSpeed + ";";
		retVal += "RightDrive1Current:" + rightDrive.getOutputCurrent() + ";";
		retVal += "RightDrive2Current:" + rightDriveSlave1.getOutputCurrent() + ";";
		retVal += "RightDrive3Current:" + rightDriveSlave2.getOutputCurrent() + ";";
		
		return retVal;
	}
}
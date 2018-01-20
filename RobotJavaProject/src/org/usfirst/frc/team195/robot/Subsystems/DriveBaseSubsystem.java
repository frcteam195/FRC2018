package org.usfirst.frc.team195.robot.Subsystems;

import java.lang.Thread;

import java.util.List;
import java.util.concurrent.Semaphore;

import org.usfirst.frc.team195.robot.Utilities.*;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class DriveBaseSubsystem extends Thread implements CustomSubsystem, Reportable {
	public static final int MIN_DRIVE_LOOP_TIME_STANDARD = 10;
	public static final int MIN_DRIVE_LOOP_TIME_MP = 3;
	public static final int MIN_SHIFT_LOOP_TIME = 3;
	public static final double DRIVE_JOYSTICK_DEADBAND = 0.1;
	
	@Override
	public void init() {
		rightDrive.setInverted(true);
		rightDriveSlave1.setInverted(true);
		rightDriveSlave2.setInverted(true);
		rightDrive.setSensorPhase(false);

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

	/*
		leftDrive.ConfigNeutralDeadband(kMotorDeadband, Constants.kTimeoutMs);
		leftDriveSlave1.ConfigNeutralDeadband(kMotorDeadband, Constants.kTimeoutMs);
		leftDriveSlave2.ConfigNeutralDeadband(kMotorDeadband, Constants.kTimeoutMs);
		rightDrive.ConfigNeutralDeadband(kMotorDeadband, Constants.kTimeoutMs);
		rightDriveSlave1.ConfigNeutralDeadband(kMotorDeadband, Constants.kTimeoutMs);
		rightDriveSlave2.ConfigNeutralDeadband(kMotorDeadband, Constants.kTimeoutMs);

		*/

		setLeftDrivePID(0.2, 0, 2, 0.06, 0);
		setRightDrivePID(0.1, 0, 2, 0.0635, 0);
		//setDrivePID(0, 0, 0, 0, 0);

		leftDrive.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
		rightDrive.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
		try {Thread.sleep(20);} catch (Exception ex) {}
	}
	
	@Override
	public synchronized void start() {
		runThread = true;
		super.start();
	}
	
	@Override
	public void subsystemHome() {
		navX.zeroYaw();
		leftDrive.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
		rightDrive.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
		try {Thread.sleep(25);} catch (Exception ex) {}
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
					
				}
			}

			ControlMode ctrlMode = leftDrive.getControlMode();

			if(ctrlMode != requestedControlMode) {
				System.out.println("Changing Control Modes!");

				switch (requestedControlMode) {
					case MotionProfile:
						leftDrive.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
						rightDrive.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
						break;
					case Velocity:
						leftDrive.set(ControlMode.Velocity, 0);
						rightDrive.set(ControlMode.Velocity, 0);
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
						System.out.println("Left Drive Underrun");
					}

					if(mpStatusRight.hasUnderrun){
						rightDrive.clearMotionProfileHasUnderrun(Constants.kTimeoutMs);
						System.out.println("Right Drive Underrun");
					}

					leftDrive.processMotionProfileBuffer();
					rightDrive.processMotionProfileBuffer();

					if (mpStatusLeft.activePointValid && mpStatusLeft.isLast) {
						leftDrive.set(ControlMode.MotionProfile, SetValueMotionProfile.Hold.value);
						System.out.println("Hold Left");
					}
					else if (startMPLeft) {
						try {
							_subsystemMutex.acquire();
							startMPLeft = false;
							_subsystemMutex.release();
						} catch (Exception ex) {

						}
						leftDrive.set(ControlMode.MotionProfile, SetValueMotionProfile.Enable.value);
						System.out.println("Enabling Left Motion Profile");
					}
					else if (mpStatusLeft.topBufferCnt == 0 && mpStatusLeft.btmBufferCnt == 0) {
						//leftDrive.Set(ControlMode.MotionProfile, SetValueMotionProfile.Disable);
						//cout << "Bad2Left" << endl;
					}

					if (mpStatusRight.activePointValid && mpStatusRight.isLast) {
						rightDrive.set(ControlMode.MotionProfile, SetValueMotionProfile.Hold.value);
						System.out.println("Hold Right");
					}
					else if (startMPRight) {
						try {
							_subsystemMutex.acquire();
							startMPRight = false;
							_subsystemMutex.release();
						} catch (Exception ex) {
							
						}
						rightDrive.set(ControlMode.MotionProfile, SetValueMotionProfile.Enable.value);
						System.out.println("Enabling Left Motion Profile");
					}
					else if (mpStatusRight.topBufferCnt == 0 && mpStatusRight.btmBufferCnt == 0) {
						//rightDrive.Set(ControlMode.MotionProfile, SetValueMotionProfile.Disable);
						//cout << "Bad2Right" << endl;
					}

					break;
				case Velocity:
					leftDrive.set(ControlMode.Velocity, leftDriveSpeed * Constants.kSensorUnitsPerRotation / 600);
					rightDrive.set(ControlMode.Velocity, rightDriveSpeed * Constants.kSensorUnitsPerRotation / 600);
					break;
				case PercentOutput:
				default:
					leftDrive.set(ControlMode.PercentOutput, leftDriveSpeed);
					rightDrive.set(ControlMode.PercentOutput, rightDriveSpeed);
					break;
			}
			
			if (holdLow) {
				shiftSol.set(DoubleSolenoid.Value.kReverse);
			} else if (highGear) {
				shiftSol.set(DoubleSolenoid.Value.kForward);
			} else {
				shiftSol.set(DoubleSolenoid.Value.kReverse);
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

	public void setDriveSpeed(DriveMotorValues d) {
		setDriveSpeed(d.leftDrive, d.rightDrive, false);
	}
	public void setDriveSpeed(double leftDriveSpeed, double rightDriveSpeed) {
		setDriveSpeed(leftDriveSpeed, rightDriveSpeed, false);
	}
	
	public void setDriveSpeed(double leftDriveSpeed, double rightDriveSpeed, boolean slowDown) {
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

		}
	}
	
	public synchronized void setHoldLowGear(boolean holdLowGear) {
		this.holdLow = holdLowGear;
	}
	
	public synchronized void setGear(boolean highGear) {
		this.highGear = highGear;
	}
	
	public void setPosition(double position) {
		try {
			_subsystemMutex.acquire();
			requestSetPosition = true;
			this.position = position;
			_subsystemMutex.release();
		} catch (Exception ex) {

		}
	}
	
	public void startMPTrajectory() {
		try {
			_subsystemMutex.acquire();
			startMPLeft = true;
			startMPRight = true;
			_subsystemMutex.release();
		} catch (Exception ex) {

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

	public void setMotionProfileTrajectory(double[][] mpLeftBuffer, double[][] mpRightBuffer) {
		this.mpLeftBuffer = mpLeftBuffer;
		this.mpRightBuffer = mpRightBuffer;
//		leftMPBufferProcess = thread(&DriveBaseSubsystem::processMPLeft, this);
//		rightMPBufferProcess = thread(&DriveBaseSubsystem::processMPRight, this);
//		if (leftMPBufferProcess.joinable())
//			leftMPBufferProcess.join();
//		if (rightMPBufferProcess.joinable())
//			rightMPBufferProcess.join();
	}
	
	public void setControlMode(ControlMode controlMode) {
		try {
			_subsystemMutex.acquire();
			requestedControlMode = controlMode;
			_subsystemMutex.release();
		} catch (Exception ex) {

		}	
	}
	
	public ControlMode getControlMode() {
		return requestedControlMode;
	}

	public void setDrivePID(double kP, double kI, double kD, double ff, int profileNum) {
		setLeftDrivePID(kP, kI, kD, ff, profileNum);
		setRightDrivePID(kP, kI, kD, ff, profileNum);
	}
	
	public void setLeftDrivePID(double kP, double kI, double kD, double ff, int profileNum) {
		leftDrive.config_kP(profileNum, kP, Constants.kTimeoutMs);
		leftDrive.config_kI(profileNum, kI, Constants.kTimeoutMs);
		leftDrive.config_kD(profileNum, kD, Constants.kTimeoutMs);
		leftDrive.config_kF(profileNum, ff, Constants.kTimeoutMs);
	}
	
	public void setRightDrivePID(double kP, double kI, double kD, double ff, int profileNum) {
		rightDrive.config_kP(profileNum, kP, Constants.kTimeoutMs);
		rightDrive.config_kI(profileNum, kI, Constants.kTimeoutMs);
		rightDrive.config_kD(profileNum, kD, Constants.kTimeoutMs);
		rightDrive.config_kF(profileNum, ff, Constants.kTimeoutMs);
	}
	public boolean isPositionWithinRange(double range) {
		return ((Math.abs(leftDrive.getSelectedSensorPosition(0)) - Math.abs(leftDriveSpeed)) < range && (Math.abs(rightDrive.getSelectedSensorPosition(0)) - Math.abs(rightDriveSpeed)) < range);
	}

	public void setMotionMagicVelocityAccel(double vel, double accel) {
		leftDrive.configMotionCruiseVelocity((int)vel, Constants.kTimeoutMs);
		leftDrive.configMotionAcceleration((int)accel, Constants.kTimeoutMs);
		rightDrive.configMotionCruiseVelocity((int)vel, Constants.kTimeoutMs);
		rightDrive.configMotionAcceleration((int)accel, Constants.kTimeoutMs);	
	}

	public static DriveBaseSubsystem getInstance() {
		if(instance == null)
			instance = new DriveBaseSubsystem();
		
		return instance;
	}
	
	public static DriveBaseSubsystem getInstance(List<CustomSubsystem> subsystemList) {
		subsystemList.add(getInstance());
		return instance;
	}
	
	@Override
	public String toString() {
		return generateReport();
	}

	private DriveBaseSubsystem() {
		super();
		ds = DriverStation.getInstance();

		Controllers robotControllers = Controllers.getInstance();
		leftDrive = robotControllers.getLeftDrive1();
		leftDriveSlave1 = robotControllers.getLeftDrive2();
		leftDriveSlave2 = robotControllers.getLeftDrive3();
		rightDrive = robotControllers.getRightDrive1();
		rightDriveSlave1 = robotControllers.getRightDrive2();
		rightDriveSlave2 = robotControllers.getRightDrive3();

		shiftSol = robotControllers.getShiftSol();

		navX = robotControllers.getNavX();

		shiftSol.set(DoubleSolenoid.Value.kReverse);
		highGear = false;

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
		
		_subsystemMutex = new Semaphore(1);
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

			}

			/*
			if (highGear)
				point.profileSlotSelect0 = 1;
			else
				point.profileSlotSelect0 = 0;
			 */

			if (i == 0)
				point.zeroPos = true;	//Set at start
			else
				point.zeroPos = false;

			if((i + 1) == mpBuffer.length)
				point.isLastPoint = true;
			else
				point.isLastPoint = false;

			talonSRX.pushMotionProfileTrajectory(point);
		}
	}

	private TrajectoryDuration GetTrajectoryDuration(int durationMs)
	{	 
		TrajectoryDuration retval = TrajectoryDuration.Trajectory_Duration_0ms;
		retval = retval.valueOf(durationMs);
		if (retval.value != durationMs) {
			DriverStation.reportError("Trajectory Duration not supported - use configMotionProfileTrajectoryPeriod instead", false);		
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
	private TalonSRX leftDriveSlave1;
	private TalonSRX leftDriveSlave2;
	private TalonSRX rightDrive;
	private TalonSRX rightDriveSlave1;
	private TalonSRX rightDriveSlave2;

	private AHRS navX;

	private DoubleSolenoid shiftSol;

	private boolean runThread;
	
	protected Semaphore _subsystemMutex;

	@Override
	public String generateReport() {
		String retVal = "";
		
		retVal += "LeftDrivePos:" + leftDrive.getSelectedSensorVelocity(0) + ";";
		retVal += "LeftDriveVel:" + leftDrive.getSelectedSensorPosition(0) + ";";
		retVal += "LeftDriveCurrent:" + leftDrive.getOutputCurrent() + ";";
		retVal += "LeftDriveOutput:" + leftDriveSpeed + ";";
		
		retVal += "RightDrivePos:" + rightDrive.getSelectedSensorVelocity(0) + ";";
		retVal += "RightDriveVel:" + rightDrive.getSelectedSensorPosition(0) + ";";
		retVal += "RightDriveCurrent:" + rightDrive.getOutputCurrent() + ";";
		retVal += "RightDriveOutput:" + rightDriveSpeed + ";";
		
		return retVal;
	}
}
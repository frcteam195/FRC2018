package org.usfirst.frc.team195.robot.Subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.*;
import org.usfirst.frc.team195.robot.Utilities.Climber.ClimberControl;
import org.usfirst.frc.team195.robot.Utilities.Drivers.TalonHelper;
import org.usfirst.frc.team195.robot.Utilities.Loops.Loop;
import org.usfirst.frc.team195.robot.Utilities.Loops.Looper;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Util;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class ClimberSubsystem implements CriticalSystemStatus, CustomSubsystem, DiagnosableSubsystem, Reportable {
	private static ClimberSubsystem instance;
	private DriverStation ds;
	private TalonSRX mClimberMotorMaster;
	private BaseMotorController mClimberMotorSlave;
	private ClimberControl mClimberControl;
	private ClimberControl mPrevClimberControl;
	private double climberPosition = 0;
	private boolean climberFault = false;


	private ClimberSubsystem() {
		ds = DriverStation.getInstance();
		Controllers robotControllers = Controllers.getInstance();

		mClimberMotorMaster = robotControllers.getClimberMotorMaster();
		mClimberMotorSlave = robotControllers.getClimberMotorSlave();

		//TODO: Set climber initial control to position once tuned
		mClimberControl = ClimberControl.OFF;
		mPrevClimberControl = ClimberControl.OFF;
	}

	public static ClimberSubsystem getInstance() {
		if(instance == null) {
			try {
				instance = new ClimberSubsystem();
			} catch (Exception ex) {
				ConsoleReporter.report(ex, MessageLevel.DEFCON1);
			}
		}

		return instance;
	}

	public static ClimberSubsystem getInstance(List<CustomSubsystem> subsystemList) {
		subsystemList.add(getInstance());
		return instance;
	}

	@Override
	public void init() {
		mClimberMotorMaster.setSensorPhase(true);

		mClimberMotorSlave.setInverted(true);

		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			setSucceeded &= mClimberMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mClimberMotorMaster.configContinuousCurrentLimit(Constants.kClimberMaxContinuousCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mClimberMotorMaster.configPeakCurrentLimit(Constants.kClimberMaxPeakCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mClimberMotorMaster.configPeakCurrentDuration(Constants.kClimberMaxPeakCurrentDurationMS, Constants.kTimeoutMs) == ErrorCode.OK;

			if (mClimberMotorSlave instanceof TalonSRX) {
				setSucceeded &= ((TalonSRX) mClimberMotorSlave).configContinuousCurrentLimit(Constants.kClimberMaxContinuousCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
				setSucceeded &= ((TalonSRX) mClimberMotorSlave).configPeakCurrentLimit(Constants.kClimberMaxPeakCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
				setSucceeded &= ((TalonSRX) mClimberMotorSlave).configPeakCurrentDuration(Constants.kClimberMaxPeakCurrentDurationMS, Constants.kTimeoutMs) == ErrorCode.OK;
				((TalonSRX) mClimberMotorSlave).enableCurrentLimit(true);
			}

			mClimberMotorMaster.enableCurrentLimit(true);

			setSucceeded &= mClimberMotorMaster.configForwardSoftLimitThreshold((int) (Constants.kClimberSoftMax * Constants.kClimberEncoderGearRatio * Constants.kSensorUnitsPerRotation), Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mClimberMotorMaster.configReverseSoftLimitThreshold((int) (Constants.kClimberSoftMin * Constants.kClimberEncoderGearRatio * Constants.kSensorUnitsPerRotation), Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mClimberMotorMaster.configForwardSoftLimitEnable(true, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mClimberMotorMaster.configReverseSoftLimitEnable(true, Constants.kTimeoutMs) == ErrorCode.OK;

		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		setSucceeded &= TalonHelper.setPIDGains(mClimberMotorMaster, 0, Constants.kClimberKp, Constants.kClimberKi, Constants.kClimberKd, Constants.kClimberKf, Constants.kClimberRampRate, Constants.kClimberIZone);
		setSucceeded &= TalonHelper.setMotionMagicParams(mClimberMotorMaster, Constants.kClimberMaxVelocity, Constants.kClimberMaxAccel);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to initialize ClimberSubsystem!!!", MessageLevel.DEFCON1);

		isSystemFaulted();
	}

	@Override
	public void subsystemHome() {
		mClimberMotorMaster.set(ControlMode.Disabled, 0);

		int homeClimberValue = (int)(Constants.kClimberSoftMax * Constants.kClimberEncoderGearRatio * Constants.kSensorUnitsPerRotation);

		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			setSucceeded &= mClimberMotorMaster.setSelectedSensorPosition(homeClimberValue, 0, Constants.kTimeoutMs) == ErrorCode.OK;

		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to zero ClimberSubsystem!!!", MessageLevel.DEFCON1);

		mClimberMotorMaster.set(ControlMode.MotionMagic, homeClimberValue);
	}


	private final Loop mLoop = new Loop() {
		@Override
		public void onFirstStart(double timestamp) {
			synchronized (ClimberSubsystem.this) {
				subsystemHome();
			}
		}

		@Override
		public void onStart(double timestamp) {
			synchronized (ClimberSubsystem.this) {

			}
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized (ClimberSubsystem.this) {
				boolean collisionOccurring = DriveBaseSubsystem.getInstance().isEmergencySafetyRequired();

				if (mClimberControl != mPrevClimberControl) {
					switch (mClimberControl) {
						case POSITION:
							mClimberMotorMaster.set(ControlMode.MotionMagic, climberPosition * Constants.kSensorUnitsPerRotation * Constants.kClimberEncoderGearRatio);
							break;
						case MANUAL:
							break;
						case OFF:
						default:
							mClimberMotorMaster.set(ControlMode.Disabled, 0);
							break;
					}
					mPrevClimberControl = mClimberControl;
				}
			}
		}
		@Override
		public void onStop(double timestamp) {

		}
	};

	@Override
	public void registerEnabledLoops(Looper in) {
		in.register(mLoop);
	}

	public synchronized void setClimberPosition(double climberPosition) {
		this.climberPosition = climberPosition;
	}

	public synchronized void setClimberControl(ClimberControl climberControl) {
		this.mClimberControl = climberControl;
	}

	@Override
	public String generateReport() {
		String retVal = "";

		retVal += "ClimberPosReq:" + climberPosition + ";";
		retVal += "ClimberPosAct:" + (mClimberMotorMaster.getSelectedSensorPosition(0) * Constants.kClimberEncoderGearRatio / Constants.kSensorUnitsPerRotation) + ";";
		retVal += "ClimberFault:" + isClimberFaulted() + ";";
		retVal += "Climber1Current:" + mClimberMotorMaster.getOutputCurrent() + ";";
		retVal += "Climber2Current:" + mClimberMotorSlave.getOutputCurrent() + ";";

		return retVal;
	}

	@Override
	public boolean runDiagnostics() {
		if (ds.isTest() && Constants.ENABLE_CLIMBER_DIAG) {
			ConsoleReporter.report("Testing Climber---------------------------------");
			final double kLowCurrentThres = Constants.kClimberTestLowCurrentThresh;
			final double kLowRpmThres = Constants.kClimberTestLowRPMThresh;

			ArrayList<MotorDiagnostics> mClimberDiagArr = new ArrayList<MotorDiagnostics>();
			mClimberDiagArr.add(new MotorDiagnostics("Climber Motor Master", mClimberMotorMaster, Constants.kClimberTestSpeed, Constants.kClimberTestDuration, false));
			mClimberDiagArr.add(new MotorDiagnostics("Climber Motor Slave", mClimberMotorSlave, mClimberMotorMaster, Constants.kClimberTestSpeed, Constants.kClimberTestDuration, false));

			boolean failure = false;

			for (MotorDiagnostics mD : mClimberDiagArr) {
				mD.runTest();

				if (mD.isCurrentUnderThreshold(kLowCurrentThres)) {
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " Current Low !!!!!!!!!!");
					failure = true;
				}

				if (mD.isRPMUnderThreshold(kLowRpmThres)) {
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " RPM Low !!!!!!!!!!");
					failure = true;
				}

				if (!mD.isSensorInPhase()) {
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " Sensor Out of Phase !!!!!!!!!!");
					failure = true;
				}

			}

			if (mClimberDiagArr.size() > 0) {
				List<Double> armMotorCurrents = mClimberDiagArr.stream().map(MotorDiagnostics::getMotorCurrent).collect(Collectors.toList());
				if (!Util.allCloseTo(armMotorCurrents, armMotorCurrents.get(0), Constants.kClimberTestCurrentDelta)) {
					failure = true;
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!! Climber Motor Currents Different !!!!!!!!!!");
				}

				List<Double> elevatorMotorRPMs = mClimberDiagArr.stream().map(MotorDiagnostics::getMotorRPM).collect(Collectors.toList());
				if (!Util.allCloseTo(elevatorMotorRPMs, elevatorMotorRPMs.get(0), Constants.kClimberTestRPMDelta)) {
					failure = true;
					ConsoleReporter.report("!!!!!!!!!!!!!!!!!!! Climber RPMs different !!!!!!!!!!!!!!!!!!!");
				}
			} else {
				ConsoleReporter.report("Climber Testing Error Occurred in system. Please check code!", MessageLevel.ERROR);
			}

			return !failure;
		} else
			return true;
	}

	public boolean isClimberFaulted() {
		return climberFault;
	}

	@Override
	public synchronized boolean isSystemFaulted() {
		boolean climberSensorPresent = mClimberMotorMaster.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;

		if (!climberSensorPresent) {
			setClimberControl(ClimberControl.OFF);

			String msg = "Could not detect encoder! \r\n\tClimber Encoder Detected: " + climberSensorPresent;
			ConsoleReporter.report(msg, MessageLevel.DEFCON1);
			DriverStation.reportError(msg, false);
		}

		climberFault = !climberSensorPresent;

		return climberFault;
	}
}

package org.usfirst.frc.team195.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.*;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.Arm.ArmConfiguration;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.Arm.PointFinder;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.Arm.PolarCoordinate;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ArmControl;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ElevatorControl;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ElevatorPosition;
import org.usfirst.frc.team195.robot.Utilities.Drivers.TalonHelper;
import org.usfirst.frc.team195.robot.Utilities.Drivers.TuneablePID;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team195.robot.Utilities.Loops.Loop;
import org.usfirst.frc.team195.robot.Utilities.Loops.Looper;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Util;

public class CubeHandlerSubsystem implements CriticalSystemStatus, CustomSubsystem, DiagnosableSubsystem, Reportable {
	private static CubeHandlerSubsystem instance;
	private IntakeControl mIntakeControl;
	private IntakeControl mPrevIntakeControl;
	private ArmControl mArmControl;
	private ArmControl mPrevArmControl;
	private ElevatorControl mElevatorControl;
	private ElevatorControl mPrevElevatorControl;

	private TuneablePID tuneableArmJoint;

	private TalonSRX mArm1Motor;
	private TalonSRX mArm2Motor;
	private TalonSRX mIntakeMotor;
	private TalonSRX mElevatorMotorMaster;
	private BaseMotorController mElevatorMotorSlave;

	private DriverStation ds;

	private ArmConfiguration armConfiguration = null;
	private ArmConfiguration mPrevArmConfiguration = new ArmConfiguration(0, 0);
	private PointFinder pointFinder;

	private boolean elevatorFault = false;
	private boolean armFault = false;

	private double elevatorHeight = 0;
	private double mPrevElevatorHeight = 0;


	private CubeHandlerSubsystem() throws Exception {
		ds = DriverStation.getInstance();
		Controllers robotControllers = Controllers.getInstance();

		mArm1Motor = robotControllers.getArm1Motor();
		mArm2Motor = robotControllers.getArm2Motor();
		mIntakeMotor = robotControllers.getIntakeMotor();
		mElevatorMotorMaster = robotControllers.getElevatorMotorMaster();
		mElevatorMotorSlave = robotControllers.getElevatorMotorSlave();

		mIntakeControl = IntakeControl.OFF;
		mPrevIntakeControl = IntakeControl.OFF;

		mArmControl = ArmControl.POSITION;
		mPrevArmControl = ArmControl.OFF;

		//TODO: Set elevator initial control to position once tuned
		mElevatorControl = ElevatorControl.OFF;
		mPrevElevatorControl = ElevatorControl.OFF;

		pointFinder = new PointFinder();
		pointFinder.setA1AngleRange(Constants.kArm1SoftMin * 360 + 1, Constants.kArm1SoftMax * 360 - 1);
		pointFinder.setA2AngleRange(Constants.kArm2SoftMin * 360 + 1, Constants.kArm2SoftMax * 360 - 1);

		//tuneableArmJoint = new TuneablePID("Arm 1 Joint", mArm1Motor, null, 5807, true, true);
		//tuneableArmJoint.start();
	}
	
	public static CubeHandlerSubsystem getInstance() {
		if(instance == null) {
			try {
				instance = new CubeHandlerSubsystem();
			} catch (Exception ex) {
				ConsoleReporter.report(ex, MessageLevel.DEFCON1);
			}
		}
		
		return instance;
	}
	
	public static CubeHandlerSubsystem getInstance(List<CustomSubsystem> subsystemList) {
		subsystemList.add(getInstance());
		return instance;
	}

	@Override
	public void init() {
		mArm1Motor.setInverted(true);
		mArm1Motor.setSensorPhase(true);

		mArm2Motor.setInverted(true);

		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			setSucceeded &= mArm1Motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArm2Motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mArm1Motor.configContinuousCurrentLimit(Constants.kArm1MaxContinuousCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArm1Motor.configPeakCurrentLimit(Constants.kArm1MaxPeakCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArm1Motor.configPeakCurrentDuration(Constants.kArm1MaxPeakCurrentDurationMS, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mArm2Motor.configContinuousCurrentLimit(Constants.kArm2MaxContinuousCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArm2Motor.configPeakCurrentLimit(Constants.kArm2MaxPeakCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArm2Motor.configPeakCurrentDuration(Constants.kArm2MaxPeakCurrentDurationMS, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mIntakeMotor.configContinuousCurrentLimit(Constants.kIntakeMaxContinuousCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mIntakeMotor.configPeakCurrentLimit(Constants.kIntakeMaxPeakCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mIntakeMotor.configPeakCurrentDuration(Constants.kIntakeMaxPeakCurrentDurationMS, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mElevatorMotorMaster.configContinuousCurrentLimit(Constants.kElevatorMaxContinuousCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mElevatorMotorMaster.configPeakCurrentLimit(Constants.kElevatorMaxPeakCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mElevatorMotorMaster.configPeakCurrentDuration(Constants.kElevatorMaxPeakCurrentDurationMS, Constants.kTimeoutMs) == ErrorCode.OK;

			if (mElevatorMotorSlave instanceof TalonSRX) {
				setSucceeded &= ((TalonSRX) mElevatorMotorSlave).configContinuousCurrentLimit(Constants.kElevatorMaxContinuousCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
				setSucceeded &= ((TalonSRX) mElevatorMotorSlave).configPeakCurrentLimit(Constants.kElevatorMaxPeakCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
				setSucceeded &= ((TalonSRX) mElevatorMotorSlave).configPeakCurrentDuration(Constants.kElevatorMaxPeakCurrentDurationMS, Constants.kTimeoutMs) == ErrorCode.OK;
				((TalonSRX) mElevatorMotorSlave).enableCurrentLimit(true);
			}

			mArm1Motor.enableCurrentLimit(true);
			mArm2Motor.enableCurrentLimit(true);
			mIntakeMotor.enableCurrentLimit(true);
			mElevatorMotorMaster.enableCurrentLimit(true);

			setSucceeded &= mArm1Motor.configForwardSoftLimitThreshold((int) (Constants.kArm1SoftMax * Constants.kArm1EncoderGearRatio * Constants.kSensorUnitsPerRotation), Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArm1Motor.configReverseSoftLimitThreshold((int) (Constants.kArm1SoftMin * Constants.kArm1EncoderGearRatio * Constants.kSensorUnitsPerRotation), Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArm1Motor.configForwardSoftLimitEnable(true, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArm1Motor.configReverseSoftLimitEnable(true, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mArm2Motor.configForwardSoftLimitThreshold((int) (Constants.kArm2SoftMax * Constants.kArm2EncoderGearRatio * Constants.kSensorUnitsPerRotation), Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArm2Motor.configReverseSoftLimitThreshold((int) (Constants.kArm2SoftMin * Constants.kArm2EncoderGearRatio * Constants.kSensorUnitsPerRotation), Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArm2Motor.configForwardSoftLimitEnable(true, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArm2Motor.configReverseSoftLimitEnable(true, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mElevatorMotorMaster.configForwardSoftLimitThreshold((int) (Constants.kElevatorSoftMax * Constants.kElevatorEncoderGearRatio * Constants.kSensorUnitsPerRotation), Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mElevatorMotorMaster.configReverseSoftLimitThreshold((int) (Constants.kElevatorSoftMin * Constants.kElevatorEncoderGearRatio * Constants.kSensorUnitsPerRotation), Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mElevatorMotorMaster.configForwardSoftLimitEnable(true, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mElevatorMotorMaster.configReverseSoftLimitEnable(true, Constants.kTimeoutMs) == ErrorCode.OK;

		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		setSucceeded &= TalonHelper.setPIDGains(mArm1Motor, 0, Constants.kArm1Kp, Constants.kArm1Ki, Constants.kArm1Kd, Constants.kArm1Kf, Constants.kArm1RampRate, Constants.kArm1IZone);
		setSucceeded &= TalonHelper.setPIDGains(mArm2Motor, 0, Constants.kArm2Kp, Constants.kArm2Ki, Constants.kArm2Kd, Constants.kArm2Kf, Constants.kArm2RampRate, Constants.kArm2IZone);
		setSucceeded &= TalonHelper.setPIDGains(mElevatorMotorMaster, 0, Constants.kElevatorKp, Constants.kElevatorKi, Constants.kElevatorKd, Constants.kElevatorKf, Constants.kElevatorRampRate, Constants.kElevatorIZone);
		setSucceeded &= TalonHelper.setMotionMagicParams(mArm1Motor, Constants.kArm1MaxVelocity, Constants.kArm1MaxAccel);
		setSucceeded &= TalonHelper.setMotionMagicParams(mArm2Motor, Constants.kArm2MaxVelocity, Constants.kArm2MaxAccel);
		setSucceeded &= TalonHelper.setMotionMagicParams(mElevatorMotorMaster, Constants.kElevatorMaxVelocity, Constants.kElevatorMaxAccel);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to initialize CubeHandlerSubsystem!!!", MessageLevel.DEFCON1);

		isSystemFaulted();
	}

	@Override
	public void subsystemHome() {
		mArm1Motor.set(ControlMode.Disabled, 0);
		mArm2Motor.set(ControlMode.Disabled, 0);
		mElevatorMotorMaster.set(ControlMode.Disabled, 0);

		int homeA1Value = (int)(Constants.kArm1SoftMax * Constants.kArm1EncoderGearRatio * Constants.kSensorUnitsPerRotation);
		int homeA2Value = (int)(Constants.kArm2SoftMin * Constants.kArm2EncoderGearRatio * Constants.kSensorUnitsPerRotation);
		int homeElevatorValue = (int)(Constants.kElevatorSoftMax * Constants.kElevatorEncoderGearRatio * Constants.kSensorUnitsPerRotation);

		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			setSucceeded &= mArm1Motor.setSelectedSensorPosition(homeA1Value, 0, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArm2Motor.setSelectedSensorPosition(homeA2Value, 0, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mElevatorMotorMaster.setSelectedSensorPosition(homeElevatorValue, 0, Constants.kTimeoutMs) == ErrorCode.OK;

		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to zero CubeHandlerSubsystem!!!", MessageLevel.DEFCON1);

		mArm1Motor.set(ControlMode.MotionMagic, homeA1Value);
		mArm2Motor.set(ControlMode.MotionMagic, homeA2Value);
		mElevatorMotorMaster.set(ControlMode.MotionMagic, homeElevatorValue);
	}

	private final Loop mLoop = new Loop() {
		@Override
		public void onFirstStart(double timestamp) {
			synchronized (CubeHandlerSubsystem.this) {
				subsystemHome();
			}
		}

		@Override
		public void onStart(double timestamp) {
			synchronized (CubeHandlerSubsystem.this) {

			}
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized (CubeHandlerSubsystem.this) {
				boolean collisionOccurring = DriveBaseSubsystem.getInstance().isCollisionOccurring();

				switch (mArmControl) {
					case POSITION:
						if (collisionOccurring) {
							setArmCoordinate(ArmConfiguration.HOME);
						}

						if (armConfiguration != null && mPrevArmConfiguration.compareTo(armConfiguration) != 0) {
							mArm1Motor.set(ControlMode.MotionMagic, QuickMaths.convertAngleToSRX(armConfiguration.getA1Angle(), Constants.kSensorUnitsPerRotation * Constants.kArm1EncoderGearRatio));
							mArm2Motor.set(ControlMode.MotionMagic, QuickMaths.convertAngleToSRX(armConfiguration.getA2Angle(), Constants.kSensorUnitsPerRotation * Constants.kArm2EncoderGearRatio));
							mPrevArmConfiguration = armConfiguration;
						}
						break;
					case MANUAL:
						break;
					case OFF:
					default:
						mArm1Motor.set(ControlMode.Disabled, 0);
						mArm2Motor.set(ControlMode.Disabled, 0);
						break;
				}
				mPrevArmControl = mArmControl;

				switch (mElevatorControl) {
					case POSITION:
						if (collisionOccurring) {
							setElevatorHeight(ElevatorPosition.HOME);
						}

						if (elevatorHeight != mPrevElevatorHeight) {
							mElevatorMotorMaster.set(ControlMode.MotionMagic, elevatorHeight * Constants.kSensorUnitsPerRotation * Constants.kElevatorEncoderGearRatio);
							mPrevElevatorHeight = elevatorHeight;
						}
						break;
					case MANUAL:
						break;
					case OFF:
					default:
						mElevatorMotorMaster.set(ControlMode.Disabled, 0);
						break;
				}
				mPrevElevatorControl = mElevatorControl;

				if (mIntakeControl != mPrevIntakeControl) {
					switch (mIntakeControl) {
						case INTAKE_IN:
							mIntakeMotor.set(ControlMode.Current, 25);
							break;
						case INTAKE_OUT:
							mIntakeMotor.set(ControlMode.Current, -55);
							break;
						case OFF:
						default:
							mIntakeMotor.set(ControlMode.Current, 0);
							break;
					}
					mPrevIntakeControl = mIntakeControl;
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


	public synchronized void setArmCoordinate(PolarCoordinate polarCoordinate) {
		armConfiguration = pointFinder.getArmConfigFromPolar(polarCoordinate);
	}

	public synchronized void setArmControl(ArmControl mArmControl) {
		this.mArmControl = mArmControl;
	}

	public synchronized void setElevatorControl(ElevatorControl elevatorControl) { this.mElevatorControl = elevatorControl; }

	public synchronized void setIntakeControl(IntakeControl mIntakeControl) {
		this.mIntakeControl = mIntakeControl;
	}

	@Override
	public String toString() {
		return generateReport();
	}

	@Override
	public String generateReport() {
		String retVal = "";

		retVal += "ElevatorPosReq:" + elevatorHeight + ";";
		retVal += "ElevatorPosAct:" + (mElevatorMotorMaster.getSelectedSensorPosition(0) * Constants.kElevatorEncoderGearRatio / Constants.kSensorUnitsPerRotation) + ";";
		retVal += "ElevatorFault:" + isElevatorFaulted() + ";";

		double a1AngleAct = (mArm1Motor.getSelectedSensorPosition(0) * Constants.kArm1EncoderGearRatio / Constants.kSensorUnitsPerRotation * 360.0);
		double a2AngleAct = (mArm2Motor.getSelectedSensorPosition(0) * Constants.kArm2EncoderGearRatio / Constants.kSensorUnitsPerRotation * 360.0);
		PolarCoordinate armActual = (new ArmConfiguration(a1AngleAct, a2AngleAct)).getPolarFromAngles();

		retVal += "Arm1PosReq:" + (armConfiguration == null ? 0 : armConfiguration.getA1Angle()) + ";";
		retVal += "Arm1PosAct:" + a1AngleAct + ";";
		retVal += "Arm2PosReq:" + (armConfiguration == null ? 0 : armConfiguration.getA2Angle()) + ";";
		retVal += "Arm2PosAct:" + a2AngleAct + ";";
		retVal += "ArmAngle:" + armActual.theta + ";";
		retVal += "ArmLength:" + armActual.r + ";";
		retVal += "ArmFault:" + isArmFaulted() + ";";

		//TODO: Add Cube Sensor
		retVal += "HasCube:" + false + ";";

		retVal += "IntakeCurrent:" + mIntakeMotor.getOutputCurrent() + ";";

		return retVal;
	}

	@Override
	public boolean runDiagnostics() {
		if (ds.isTest() && Constants.ENABLE_CUBE_HANDLER_DIAG) {
			ConsoleReporter.report("Testing CubeHandler---------------------------------");
			boolean testPassed = true;
			testPassed &= runArmDiagnostics();
			testPassed &= runElevatorDiagnostics();
			testPassed &= runIntakeDiagnostics();
			return testPassed;
		} else
			return true;
	}

	private boolean runArmDiagnostics() {
		ConsoleReporter.report("Testing Arm---------------------------------");
		final double kLowCurrentThres = 0.5;
		final double kLowRpmThres = 200;

		ArrayList<MotorDiagnostics> mArmDiagArr = new ArrayList<MotorDiagnostics>();
		mArmDiagArr.add(new MotorDiagnostics("Arm Joint 1", mArm1Motor, 0.3, 1, true));
		mArmDiagArr.add(new MotorDiagnostics("Arm Joint 2", mArm2Motor, 0.2, 1, false));

		boolean failure = false;

		for (MotorDiagnostics mD : mArmDiagArr) {
			mD.runTest();

			if (mD.isCurrentUnderThreshold(kLowCurrentThres)) {
				ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " Current Low !!!!!!!!!!");
				failure = true;
			}

			if (mD.isRPMUnderThreshold(kLowRpmThres)) {
				ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " RPM Low !!!!!!!!!!");
				failure = true;
			}

		}

		if (mArmDiagArr.size() > 0) {
			List<Double> armMotorCurrents = mArmDiagArr.stream().map(MotorDiagnostics::getMotorCurrent).collect(Collectors.toList());
			if (!Util.allCloseTo(armMotorCurrents, armMotorCurrents.get(0), 5.0)) {
				failure = true;
				ConsoleReporter.report("!!!!!!!!!!!!!!!!!! Arm Motor Currents Different !!!!!!!!!!");
			}

		} else {
			ConsoleReporter.report("Arm Testing Error Occurred in system. Please check code!", MessageLevel.ERROR);
		}

		return !failure;
	}

	private boolean runElevatorDiagnostics() {
		ConsoleReporter.report("Testing Elevator---------------------------------");
		final double kLowCurrentThres = 0.5;
		final double kLowRpmThres = 200;

		ArrayList<MotorDiagnostics> mElevatorDiagArr = new ArrayList<MotorDiagnostics>();
		mElevatorDiagArr.add(new MotorDiagnostics("Elevator Motor Master", mElevatorMotorMaster, 0.3, 1, false));
		mElevatorDiagArr.add(new MotorDiagnostics("Elevator Motor Slave", mElevatorMotorSlave, mElevatorMotorMaster, 0.3, 1, false));

		boolean failure = false;

		for (MotorDiagnostics mD : mElevatorDiagArr) {
			mD.runTest();

			if (mD.isCurrentUnderThreshold(kLowCurrentThres)) {
				ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " Current Low !!!!!!!!!!");
				failure = true;
			}

			if (mD.isRPMUnderThreshold(kLowRpmThres)) {
				ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " RPM Low !!!!!!!!!!");
				failure = true;
			}

		}

		if (mElevatorDiagArr.size() > 0) {
			List<Double> armMotorCurrents = mElevatorDiagArr.stream().map(MotorDiagnostics::getMotorCurrent).collect(Collectors.toList());
			if (!Util.allCloseTo(armMotorCurrents, armMotorCurrents.get(0), 5.0)) {
				failure = true;
				ConsoleReporter.report("!!!!!!!!!!!!!!!!!! Elevator Motor Currents Different !!!!!!!!!!");
			}

			List<Double> elevatorMotorRPMs = mElevatorDiagArr.stream().map(MotorDiagnostics::getMotorRPM).collect(Collectors.toList());
			if (!Util.allCloseTo(elevatorMotorRPMs, elevatorMotorRPMs.get(0), 40)) {
				failure = true;
				ConsoleReporter.report("!!!!!!!!!!!!!!!!!!! Elevator RPMs different !!!!!!!!!!!!!!!!!!!");
			}
		} else {
			ConsoleReporter.report("Arm Testing Error Occurred in system. Please check code!", MessageLevel.ERROR);
		}

		return !failure;
	}

	private boolean runIntakeDiagnostics() {
		ConsoleReporter.report("Testing Intake---------------------------------");
		final double kLowCurrentThres = 0.5;

		ArrayList<MotorDiagnostics> mIntakeDiagArr = new ArrayList<MotorDiagnostics>();
		mIntakeDiagArr.add(new MotorDiagnostics("Intake", mIntakeMotor, 1));

		boolean failure = false;

		for (MotorDiagnostics mD : mIntakeDiagArr) {
			mD.runTest();

			if (mD.isCurrentUnderThreshold(kLowCurrentThres)) {
				ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " Current Low !!!!!!!!!!");
				failure = true;
			}
		}

		return !failure;
	}

	private synchronized boolean checkIfArmIsFaulted() {
		boolean allSensorsPresent = true;

		boolean arm1SensorPresent = mArm1Motor.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;
		boolean arm2SensorPresent = mArm2Motor.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;
		allSensorsPresent &= arm1SensorPresent;
		allSensorsPresent &= arm2SensorPresent;

		if (!allSensorsPresent) {
			setArmControl(ArmControl.OFF);

			String msg = "Could not detect encoder! \r\n\tArm 1 Encoder Detected: " + arm1SensorPresent + "\r\n\tArm 2 Encoder Detected: " + arm2SensorPresent;
			ConsoleReporter.report(msg, MessageLevel.DEFCON1);
			DriverStation.reportError(msg, false);
		}
		armFault = !allSensorsPresent;
		return armFault;
	}

	public boolean isArmFaulted() {
		return armFault;
	}

	private synchronized boolean checkIfElevatorIsFaulted() {
		boolean elevatorSensorPresent = mElevatorMotorMaster.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;

		if (!elevatorSensorPresent) {
			setElevatorControl(ElevatorControl.OFF);

			String msg = "Could not detect encoder! \r\n\tElevator Encoder Detected: " + elevatorSensorPresent;
			ConsoleReporter.report(msg, MessageLevel.DEFCON1);
			DriverStation.reportError(msg, false);
		}

		elevatorFault = !elevatorSensorPresent;
		return elevatorFault;
	}

	public boolean isElevatorFaulted() {
		return elevatorFault;
	}

	@Override
	public boolean isSystemFaulted() {
		boolean allSensorsPresent = true;

		allSensorsPresent &= !checkIfArmIsFaulted();
		allSensorsPresent &= !checkIfElevatorIsFaulted();

		return !allSensorsPresent;
	}

	public synchronized void setElevatorHeight(double elevatorHeight) {
		this.elevatorHeight = elevatorHeight;
	}
}


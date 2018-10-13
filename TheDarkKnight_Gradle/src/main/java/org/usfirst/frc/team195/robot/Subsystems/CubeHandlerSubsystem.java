package org.usfirst.frc.team195.robot.Subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team195.robot.AutoSelectionReceiver;
import org.usfirst.frc.team195.robot.LEDController;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.DashboardReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.*;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.*;
import org.usfirst.frc.team195.robot.Utilities.Drivers.CKTalonSRX;
import org.usfirst.frc.team195.robot.Utilities.Drivers.KnightDigitalInput;
import org.usfirst.frc.team195.robot.Utilities.Drivers.TalonHelper;
import org.usfirst.frc.team195.robot.Utilities.Drivers.TuneablePID;
import org.usfirst.frc.team195.robot.Utilities.Loops.Loop;
import org.usfirst.frc.team195.robot.Utilities.Loops.Looper;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Util;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.stream.Collectors;

public class CubeHandlerSubsystem implements CriticalSystemStatus, CustomSubsystem, DiagnosableSubsystem, Reportable {
	private static final int kElevatorUpRateSlot = 0;
	private static final int kElevatorDownRateSlot = 1;
	private static final int kArmNormalRateSlot = 0;
	private static final int kArmFastRateSlot = 1;
	private static CubeHandlerSubsystem instance;
	private IntakeControl mIntakeControl;
	private IntakeControl mPrevIntakeControl;
	private ArmControl mArmControl;
	private ArmControl mPrevArmControl;
	private ElevatorControl mElevatorControl;
	private ElevatorControl mPrevElevatorControl;

	private TuneablePID tuneableArmJoint;
	private TuneablePID tuneableElevator;

	private CKTalonSRX mArmMotor;
	private TalonSRX mIntake2Motor;
	private TalonSRX mIntakeMotor;
	private CKTalonSRX mElevatorMotorMaster;
	private BaseMotorController mElevatorMotorSlave;
	private BaseMotorController mElevatorMotorSlave2;
	private BaseMotorController mElevatorMotorSlave3;

	private Solenoid intakeSolenoid;

	private DriverStation ds;

	private KnightDigitalInput mElevatorHomeSwitch;
	private KnightDigitalInput mCubeSensor;

	private LEDController ledController;

	private boolean elevatorFault = false;
	private boolean armFault = false;
	private boolean intakeFault = false;

	private double elevatorHeight = 0;	//Value in rotations of output shaft
	private double mPrevElevatorHeight = 0;
	private double currentOverageCounter = 0;
	private double elevatorHomingTimeStart = 0;
	private double armHomingTimeStart = 0;
	private double armRotation = 0;	//Value in Degrees
	private double mPrevArmRotation = 0;
	private int armEncoderLossCounter = 0;

	private double liftArmTimerStart = 0;
	private boolean requestLiftArmForCube = false;
	private double armOpenLoopDriveVal = 0;
	private double mPrevArmOpenLoopDriveVal = 0;

	private double mPrevArmEncoderVal = 0;

	private double mPrevIntakeCurrentOutput = 0;

	private boolean disableCollisionPrevention = false;
	private boolean disableFastDown = false;

	private boolean requestElevatorHoming = false;
	private double elevatorRequestHomingTime = 0;

	private double elevatorOpenLoopVal = 0;

	private boolean blinkOnHome = true;

	private boolean autoMode = true;

	private CubeHandlerSubsystem() throws Exception {
		ds = DriverStation.getInstance();
		Controllers robotControllers = Controllers.getInstance();
		ledController = LEDController.getInstance();

		mArmMotor = robotControllers.getArm1Motor();
		mIntakeMotor = robotControllers.getIntakeMotor();
		mIntake2Motor = robotControllers.getIntake2Motor();
		mElevatorMotorMaster = robotControllers.getElevatorMotorMaster();
		mElevatorMotorSlave = robotControllers.getElevatorMotorSlave();
		mElevatorMotorSlave2 = robotControllers.getElevatorMotorSlave2();
		mElevatorMotorSlave3 = robotControllers.getElevatorMotorSlave3();

		mElevatorHomeSwitch = robotControllers.getElevatorHomeSwitch();
		mCubeSensor = robotControllers.getCubeSensor();

		intakeSolenoid = robotControllers.getIntakeSolenoid();

		mIntakeControl = IntakeControl.OFF;
		mPrevIntakeControl = IntakeControl.OFF;

		mArmControl = ArmControl.POSITION;
		mPrevArmControl = ArmControl.OFF;

		mElevatorControl = ElevatorControl.POSITION;
		mPrevElevatorControl = ElevatorControl.OFF;


//		tuneableArmJoint = new TuneablePID("Arm 1 Joint", mArmMotor, null, 5807, true, true);
//		tuneableArmJoint.start();
//		tuneableElevator = new TuneablePID("Elevator", mElevatorMotorMaster, null, 5807, true, true);
//		tuneableElevator.start();
//		mElevatorMotorMaster.set(ControlMode.MotionMagic, mElevatorMotorMaster.getSelectedSensorPosition(0));
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
		mArmMotor.setInverted(true);
		mArmMotor.setSensorPhase(true);
		mArmMotor.setNeutralMode(NeutralMode.Brake);

		mElevatorMotorMaster.setSensorPhase(false);
		mElevatorMotorMaster.setInverted(true);
		mElevatorMotorSlave3.setInverted(true);
		mElevatorMotorMaster.setNeutralMode(NeutralMode.Brake);
		mElevatorMotorSlave.setNeutralMode(NeutralMode.Brake);
		mElevatorMotorSlave2.setNeutralMode(NeutralMode.Brake);
		mElevatorMotorSlave3.setNeutralMode(NeutralMode.Brake);

		mIntakeMotor.setInverted(false);
		mIntake2Motor.setInverted(false);
		mIntakeMotor.setNeutralMode(NeutralMode.Brake);
		mIntake2Motor.setNeutralMode(NeutralMode.Brake);

		setIntakeClamp(false);

		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			setSucceeded &= mArmMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs) == ErrorCode.OK;

			//setSucceeded &= mIntake2Motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs) == ErrorCode.OK;
			//setSucceeded &= mArmMotor.configRemoteFeedbackFilter(Constants.kIntake2MotorId, RemoteSensorSource.TalonSRX_SelectedSensor, 0, Constants.kTimeoutMs) == ErrorCode.OK;
			//setSucceeded &= mArmMotor.configRemoteFeedbackFilter(0x00, RemoteSensorSource.Off, 1, Constants.kTimeoutMs) == ErrorCode.OK;
			//setSucceeded &= mArmMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mArmMotor.configContinuousCurrentLimit(Constants.kArmMaxContinuousCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArmMotor.configPeakCurrentLimit(Constants.kArmMaxPeakCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArmMotor.configPeakCurrentDuration(Constants.kArmMaxPeakCurrentDurationMS, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mArmMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 80, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mIntakeMotor.configContinuousCurrentLimit(Constants.kIntakeMaxContinuousCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mIntakeMotor.configPeakCurrentLimit(Constants.kIntakeMaxPeakCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mIntakeMotor.configPeakCurrentDuration(Constants.kIntakeMaxPeakCurrentDurationMS, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mIntakeMotor.configOpenloopRamp(Constants.kIntakeOLRampRate, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mIntake2Motor.configContinuousCurrentLimit(Constants.kIntakeMaxContinuousCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mIntake2Motor.configPeakCurrentLimit(Constants.kIntakeMaxPeakCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mIntake2Motor.configPeakCurrentDuration(Constants.kIntakeMaxPeakCurrentDurationMS, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mIntake2Motor.configOpenloopRamp(Constants.kIntakeOLRampRate, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mElevatorMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mElevatorMotorMaster.configContinuousCurrentLimit(Constants.kElevatorMaxContinuousCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mElevatorMotorMaster.configPeakCurrentLimit(Constants.kElevatorMaxPeakCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mElevatorMotorMaster.configPeakCurrentDuration(Constants.kElevatorMaxPeakCurrentDurationMS, Constants.kTimeoutMs) == ErrorCode.OK;

			if (mElevatorMotorSlave instanceof TalonSRX) {
				setSucceeded &= ((TalonSRX) mElevatorMotorSlave).configContinuousCurrentLimit(Constants.kElevatorMaxContinuousCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
				setSucceeded &= ((TalonSRX) mElevatorMotorSlave).configPeakCurrentLimit(Constants.kElevatorMaxPeakCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
				setSucceeded &= ((TalonSRX) mElevatorMotorSlave).configPeakCurrentDuration(Constants.kElevatorMaxPeakCurrentDurationMS, Constants.kTimeoutMs) == ErrorCode.OK;
				((TalonSRX) mElevatorMotorSlave).enableCurrentLimit(true);
			}

			if (mElevatorMotorSlave2 instanceof TalonSRX) {
				setSucceeded &= ((TalonSRX) mElevatorMotorSlave2).configContinuousCurrentLimit(Constants.kElevatorMaxContinuousCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
				setSucceeded &= ((TalonSRX) mElevatorMotorSlave2).configPeakCurrentLimit(Constants.kElevatorMaxPeakCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
				setSucceeded &= ((TalonSRX) mElevatorMotorSlave2).configPeakCurrentDuration(Constants.kElevatorMaxPeakCurrentDurationMS, Constants.kTimeoutMs) == ErrorCode.OK;
				((TalonSRX) mElevatorMotorSlave2).enableCurrentLimit(true);
			}

			if (mElevatorMotorSlave3 instanceof TalonSRX) {
				setSucceeded &= ((TalonSRX) mElevatorMotorSlave3).configContinuousCurrentLimit(Constants.kElevatorMaxContinuousCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
				setSucceeded &= ((TalonSRX) mElevatorMotorSlave3).configPeakCurrentLimit(Constants.kElevatorMaxPeakCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
				setSucceeded &= ((TalonSRX) mElevatorMotorSlave3).configPeakCurrentDuration(Constants.kElevatorMaxPeakCurrentDurationMS, Constants.kTimeoutMs) == ErrorCode.OK;
				((TalonSRX) mElevatorMotorSlave3).enableCurrentLimit(true);
			}

			mArmMotor.enableCurrentLimit(true);
			mIntake2Motor.enableCurrentLimit(false);
			mIntakeMotor.enableCurrentLimit(false);
			mElevatorMotorMaster.enableCurrentLimit(true);

			setSucceeded &= mIntakeMotor.configForwardSoftLimitEnable(false, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mIntake2Motor.configReverseSoftLimitEnable(false, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mArmMotor.configForwardSoftLimitThreshold((int) (Constants.kArmSoftMax * Constants.kArmEncoderGearRatio * Constants.kSensorUnitsPerRotation), Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArmMotor.configReverseSoftLimitThreshold((int) (Constants.kArmSoftMin * Constants.kArmEncoderGearRatio * Constants.kSensorUnitsPerRotation), Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArmMotor.configForwardSoftLimitEnable(true, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArmMotor.configReverseSoftLimitEnable(true, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArmMotor.configAllowableClosedloopError(0, Constants.kArmAllowedError, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mElevatorMotorMaster.configForwardSoftLimitThreshold((int) (Constants.kElevatorSoftMax * Constants.kElevatorEncoderGearRatio * Constants.kSensorUnitsPerRotation), Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mElevatorMotorMaster.configReverseSoftLimitThreshold((int) (Constants.kElevatorSoftMin * Constants.kElevatorEncoderGearRatio * Constants.kSensorUnitsPerRotation), Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mElevatorMotorMaster.configForwardSoftLimitEnable(true, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mElevatorMotorMaster.configReverseSoftLimitEnable(true, Constants.kTimeoutMs) == ErrorCode.OK;

		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		mElevatorMotorMaster.selectProfileSlot(kElevatorUpRateSlot, 0);
		setSucceeded &= TalonHelper.setPIDGains(mElevatorMotorMaster, kElevatorUpRateSlot, Constants.kElevatorKp, Constants.kElevatorKi, Constants.kElevatorKd, Constants.kElevatorKf, Constants.kElevatorRampRate, Constants.kElevatorIZone);
		setSucceeded &= TalonHelper.setPIDGains(mElevatorMotorMaster, kElevatorDownRateSlot, Constants.kElevatorKp, Constants.kElevatorKi, Constants.kElevatorKd, Constants.kElevatorKf, Constants.kElevatorRampRate, Constants.kElevatorIZone);
		setSucceeded &= TalonHelper.setMotionMagicParams(mElevatorMotorMaster, kElevatorUpRateSlot, Constants.kElevatorMaxVelocityUp, Constants.kElevatorMaxAccelUp);
		setSucceeded &= TalonHelper.setMotionMagicParams(mElevatorMotorMaster, kElevatorDownRateSlot, Constants.kElevatorMaxVelocityDown, Constants.kElevatorMaxAccelDown);
		mElevatorMotorMaster.selectProfileSlot(kElevatorUpRateSlot, 0);


		mArmMotor.selectProfileSlot(kArmNormalRateSlot, 0);
		setSucceeded &= TalonHelper.setPIDGains(mArmMotor, kArmNormalRateSlot, Constants.kArmKp, Constants.kArmKi, Constants.kArmKd, Constants.kArmKf, Constants.kArmRampRate, Constants.kArmIZone);
		setSucceeded &= TalonHelper.setPIDGains(mArmMotor, kArmFastRateSlot, Constants.kArmKp, Constants.kArmKi, Constants.kArmKd, Constants.kArmKf, Constants.kArmRampRate, Constants.kArmIZone);
		setSucceeded &= TalonHelper.setMotionMagicParams(mArmMotor, kArmNormalRateSlot, Constants.kArmMaxVelocity, Constants.kArmMaxAccel);
		setSucceeded &= TalonHelper.setMotionMagicParams(mArmMotor, kArmFastRateSlot, Constants.kArmMaxVelocity, Constants.kArmMaxAccelDownFast);

		setSucceeded &= TalonHelper.setPIDGains(mIntakeMotor, 0, Constants.kIntakeKp, Constants.kIntakeKi, Constants.kIntakeKd, Constants.kIntakeKf, Constants.kIntakeCLRampRate, Constants.kIntakeIZone);
		setSucceeded &= TalonHelper.setPIDGains(mIntake2Motor,  0, Constants.kIntakeKp, Constants.kIntakeKi, Constants.kIntakeKd, Constants.kIntakeKf, Constants.kIntakeCLRampRate, Constants.kIntakeIZone);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to initialize CubeHandlerSubsystem!!!", MessageLevel.DEFCON1);

		isSystemFaulted();
	}

	@Override
	public void subsystemHome() {
		boolean setSucceeded = true;
		setSucceeded &= zeroElevator();
		setSucceeded &= zeroArm();
	}

	private boolean zeroElevator() {
		return zeroElevator(false);
	}

	private boolean zeroElevator(boolean enterRehomingMode) {
		mElevatorMotorMaster.set(ControlMode.Disabled, 0);
		int homeElevatorValue = (int)(Constants.kElevatorHome * Constants.kElevatorEncoderGearRatio * Constants.kSensorUnitsPerRotation);

		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			setSucceeded &= mElevatorMotorMaster.setSelectedSensorPosition(homeElevatorValue, 0, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mElevatorMotorMaster.configReverseSoftLimitEnable(true, Constants.kTimeoutMs) == ErrorCode.OK;

		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to zero Elevator!!!", MessageLevel.DEFCON1);

		mElevatorMotorMaster.set(ControlMode.MotionMagic, homeElevatorValue);
		double newPos = ElevatorPosition.TRUE_HOME;
		newPos += enterRehomingMode ? ElevatorPosition.TENSION_OFFSET : ElevatorPosition.TENSION_OFFSET;
		setElevatorHeight(newPos);

		if (enterRehomingMode) {
			elevatorRequestHomingTime = Timer.getFPGATimestamp();
			setElevatorRequestHoming(true);
		}

		setElevatorFaulted(false, false);

		return retryCounter < Constants.kTalonRetryCount && setSucceeded;
	}

	private boolean zeroArm() {
		return zeroArm((int)(Constants.kArmHomingSetpoint * Constants.kSensorUnitsPerRotation * Constants.kArmEncoderGearRatio));
	}

	private boolean zeroArm(int homeArmValue) {
		setArmControl(ArmControl.OFF);
		mArmMotor.set(ControlMode.Disabled, 0);

		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			//setSucceeded &= mIntake2Motor.setSelectedSensorPosition(homeArmValue, 0, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mArmMotor.setSelectedSensorPosition(homeArmValue, 0, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArmMotor.configForwardSoftLimitEnable(true, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArmMotor.configReverseSoftLimitEnable(true, Constants.kTimeoutMs) == ErrorCode.OK;

		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to zero Arm!!!", MessageLevel.DEFCON1);

		mArmMotor.set(ControlMode.MotionMagic, homeArmValue);
		setArmRotationDeg(homeArmValue / Constants.kSensorUnitsPerRotation / Constants.kArmEncoderGearRatio / Constants.kArmFinalRotationsPerDegree);
		setArmControl(ArmControl.POSITION);
		setArmFaulted(false, false);
		return retryCounter < Constants.kTalonRetryCount && setSucceeded;
	}

	public boolean isAuto() {
		return autoMode;
	}

	private synchronized void setAuto(boolean autoMode) {
		this.autoMode = autoMode;
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
		public void onLoop(double timestamp, boolean isAuto) {
			synchronized (CubeHandlerSubsystem.this) {
				boolean collisionOccurring = DriveBaseSubsystem.getInstance().isEmergencySafetyRequired();
//				SmartDashboard.putBoolean("ElevatorHomeSwitch", mElevatorHomeSwitch.get());
//				SmartDashboard.putString("ElevatorControlMode", mElevatorControl.toString());
//				SmartDashboard.putString("ArmControlMode", mArmControl.toString());
//				ConsoleReporter.report("Disable collision: " + isCollisionPreventionDisabled());
//				SmartDashboard.putBoolean("DisableCollisionPrevention", isCollisionPreventionDisabled());

//				SmartDashboard.putString("Elevator Position", "" + mElevatorMotorMaster.getSelectedSensorPosition(0)/4096.0);
//				SmartDashboard.putBoolean("Homing switch: ", mElevatorHomeSwitch.get());
				setAuto(isAuto);

				switch (mArmControl) {
					case POSITION:

						if (collisionOccurring && !isAuto) {

						}

//						SmartDashboard.putNumber("ArmRequest", armRotation);

						//Collision interference avoidance
						//Check both, actual, and requested
						double tmpArmRotation = (elevatorHeight >= ElevatorPosition.ARM_COLLISION_POINT) || disableCollisionPrevention ? armRotation :
								Util.limit(armRotation, 0, Constants.kArmHomingSetpoint / Constants.kArmFinalRotationsPerDegree);
						tmpArmRotation = (getElevatorHeight() >= ElevatorPosition.ARM_COLLISION_POINT) || disableCollisionPrevention ? tmpArmRotation :
								Util.limit(tmpArmRotation, 0, Constants.kArmHomingSetpoint / Constants.kArmFinalRotationsPerDegree);
//						SmartDashboard.putNumber("ArmTmp", tmpArmRotation);
						if (tmpArmRotation != mPrevArmRotation) {
							if (tmpArmRotation < getArmRotationDeg() && !hasCube())
								mArmMotor.set(ControlMode.MotionMagic, tmpArmRotation * Constants.kArmFinalRotationsPerDegree * Constants.kSensorUnitsPerRotation * Constants.kArmEncoderGearRatio, kArmFastRateSlot);
							else
								mArmMotor.set(ControlMode.MotionMagic, tmpArmRotation * Constants.kArmFinalRotationsPerDegree * Constants.kSensorUnitsPerRotation * Constants.kArmEncoderGearRatio, kArmNormalRateSlot);
							mPrevArmRotation = tmpArmRotation;
						}

						break;
					case OPEN_LOOP:
						if (armOpenLoopDriveVal != mPrevArmOpenLoopDriveVal) {
							mArmMotor.configForwardSoftLimitEnable(false, Constants.kTimeoutMsFast);
							mArmMotor.configReverseSoftLimitEnable(false, Constants.kTimeoutMsFast);
							mArmMotor.set(ControlMode.PercentOutput, armOpenLoopDriveVal);
							mPrevArmOpenLoopDriveVal = armOpenLoopDriveVal;
						}
						break;
					case HOMING:
						if (mPrevArmControl != ArmControl.HOMING)
							armHomingTimeStart = Timer.getFPGATimestamp();

						zeroArm(-5);
//						setArmRotationDeg(ArmPosition.DOWN);
//						setArmControl(ArmControl.POSITION);

						if (Timer.getFPGATimestamp() - armHomingTimeStart > Constants.kArmHomingTimeout) {
							setArmControl(ArmControl.OPEN_LOOP);
							ConsoleReporter.report("Arm Failed to Home! Arm Disabled!", MessageLevel.DEFCON1);
						}

						break;
					case OFF:
					default:
						if (mArmMotor.getControlMode() != ControlMode.Disabled)
							mArmMotor.set(ControlMode.Disabled, 0);
						break;
				}
				mPrevArmControl = mArmControl;

				switch (mElevatorControl) {
					case POSITION:
						if (collisionOccurring && !isAuto) {
							//setElevatorHeight(ElevatorPosition.HOME);
						}
//
//						if (elevatorHeight <= ElevatorPosition.HOME &&
//								Math.abs(QuickMaths.convertNativeUnitsToRotations(mElevatorMotorMaster.getSelectedSensorPosition(0)) - elevatorHeight)
//										< Constants.kElevatorDeviationThreshold &&
//								mElevatorHomeSwitch.get() && !isAuto) {
//							setElevatorControl(ElevatorControl.HOMING);
//							break;
//						}

						//Collision interference avoidance
						//Check arm actual
						double tmpElevatorHeight = (getArmRotationDeg() <= ArmPosition.ELEVATOR_COLLISION_POINT) || disableCollisionPrevention ? elevatorHeight :
								Util.limit(elevatorHeight, ElevatorPosition.ARM_COLLISION_POINT - Constants.kElevatorDeviationThreshold, Constants.kElevatorSoftMax);

						if (!mElevatorHomeSwitch.get() && elevatorHeight <= ElevatorPosition.TRUE_HOME) {
							zeroElevator(true);
							//setElevatorHeight(getElevatorHeight() + Constants.kElevatorSafetyDelta * 2);
						} else if (tmpElevatorHeight != mPrevElevatorHeight) {
							if ((getElevatorHeight() < tmpElevatorHeight) || isAuto || disableFastDown || true)
								mElevatorMotorMaster.set(ControlMode.MotionMagic, tmpElevatorHeight * Constants.kSensorUnitsPerRotation * Constants.kElevatorEncoderGearRatio, kElevatorUpRateSlot);
							else
								mElevatorMotorMaster.set(ControlMode.MotionMagic, tmpElevatorHeight * Constants.kSensorUnitsPerRotation * Constants.kElevatorEncoderGearRatio, kElevatorDownRateSlot);

							mPrevElevatorHeight = tmpElevatorHeight;
						}

						if (requestElevatorHoming && ((timestamp - elevatorRequestHomingTime) >= Constants.kElevatorRetensioningTime)) {
							setElevatorRequestHoming(false);
							setElevatorControl(ElevatorControl.HOMING);
							elevatorHomingTimeStart = Timer.getFPGATimestamp();
							break;
						}

						break;
					case MANUAL:
						mElevatorMotorMaster.set(ControlMode.PercentOutput, elevatorOpenLoopVal);
						break;
					case HOMING:
						if (mPrevElevatorControl != ElevatorControl.HOMING) {
							elevatorHomingTimeStart = Timer.getFPGATimestamp();
						}

						mElevatorMotorMaster.configReverseSoftLimitEnable(false, Constants.kTimeoutMsFast);
						mElevatorMotorMaster.set(ControlMode.PercentOutput, Constants.kElevatorHomingSpeed);
						if (!mElevatorHomeSwitch.get()) {
							zeroElevator();
							setElevatorControl(ElevatorControl.POSITION);
							//setElevatorHeight(getElevatorHeight() + Constants.kElevatorSafetyDelta * 2);
							if (blinkOnHome) {
								ledController.configureBlink(3, LEDController.kDefaultBlinkDuration);
								ledController.setLEDColor(Constants.kElevatorHomeColor);
								ledController.setRequestedState(LEDController.LEDState.BLINK);
							}
						}

						if (Timer.getFPGATimestamp() - elevatorHomingTimeStart > Constants.kElevatorHomingTimeout) {
							//setElevatorControl(ElevatorControl.OFF);
							zeroElevator();
							setElevatorControl(ElevatorControl.POSITION);
							setElevatorFaulted(true, false);
							ConsoleReporter.report("Elevator Failed to Home! Elevator Disabled!", MessageLevel.DEFCON1);
						}
						break;
					case OFF:
					default:
						if (mElevatorMotorMaster.getControlMode() != ControlMode.Disabled)
							mElevatorMotorMaster.set(ControlMode.Disabled, 0);
						break;
				}
				mPrevElevatorControl = mElevatorControl;


				if (mIntakeControl != mPrevIntakeControl || mIntakeControl == IntakeControl.OFF || mIntakeControl == IntakeControl.HOLD) {
					switch (mIntakeControl) {
						case INTAKE_IN:
							mIntakeMotor.set(ControlMode.PercentOutput, 1);
							mIntake2Motor.set(ControlMode.PercentOutput, 1);
							break;
						case INTAKE_OUT_EXTRA_FAST:
							mIntakeMotor.set(ControlMode.PercentOutput, -1);
							mIntake2Motor.set(ControlMode.PercentOutput, -1);
							break;
						case INTAKE_OUT_AUTO:
							mIntakeMotor.set(ControlMode.PercentOutput, -0.80);
							mIntake2Motor.set(ControlMode.PercentOutput, -0.80);
							break;
						case INTAKE_OUT:
							mIntakeMotor.set(ControlMode.PercentOutput, -0.65);
							mIntake2Motor.set(ControlMode.PercentOutput, -0.65);
							break;
						case INTAKE_OUT_HALFSPEED:
							mIntakeMotor.set(ControlMode.PercentOutput, -0.4);
							mIntake2Motor.set(ControlMode.PercentOutput, -0.4);
							break;
						case INTAKE_OUT_SLOW:
							mIntakeMotor.set(ControlMode.PercentOutput, -0.3);
							mIntake2Motor.set(ControlMode.PercentOutput, -0.3);
							break;
						case INTAKE_OUT_A_LITTLE_LESS:
							mIntakeMotor.set(ControlMode.PercentOutput, -0.6);
							mIntake2Motor.set(ControlMode.PercentOutput, -0.6);
							break;
						case HOLD:
						case OFF:
						default:
							if (hasCube() && !intakeSolenoid.get() && (armRotation == ArmPosition.VERTICAL || isAuto)) {
								mIntakeMotor.set(ControlMode.Current, Constants.kIntakeHoldCurrent);
								mIntake2Motor.set(ControlMode.Current, Constants.kIntakeHoldCurrent);
							} else if (mIntakeMotor.getControlMode() != ControlMode.Disabled || mIntake2Motor.getControlMode() != ControlMode.Disabled) {
								mIntakeMotor.set(ControlMode.Disabled, 0);
								mIntake2Motor.set(ControlMode.Disabled, 0);
							}
							break;
					}
					mPrevIntakeControl = mIntakeControl;
				}

				if (mCubeSensor.getRisingEdge() && intakeSolenoid.get() && getElevatorHeight() < ElevatorPosition.PICKUP_CUBE_THRESHOLD) {
					setIntakeClamp(false);

					ledController.configureBlink(3, LEDController.kDefaultBlinkDuration);
					ledController.setLEDColor(Constants.kGotCubeColor);
					ledController.setRequestedState(LEDController.LEDState.BLINK);

					if (!isAuto) {
						liftArmTimerStart = Timer.getFPGATimestamp();

						if (elevatorHeight <= ElevatorPosition.SENSOR_LIFT_OFFSET && !Controllers.getInstance().getButtonBox1().getRawButton(Constants.BB1_ARM_DOWN))
							requestLiftArmForCube = true;
					}
				}

				if (requestLiftArmForCube && (Timer.getFPGATimestamp() - liftArmTimerStart) > 0.5 && !isAuto && mCubeSensor.get()) {
					setArmRotationDeg(ArmPosition.VERTICAL);
					requestLiftArmForCube = false;
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

	private synchronized void setElevatorRequestHoming(boolean requestElevatorHoming) {
		this.requestElevatorHoming = requestElevatorHoming;
	}

	public synchronized void setArmOpenLoopDriveVal(double val) {
		this.armOpenLoopDriveVal = val;
	}

	public synchronized void setIntakeClamp(boolean open) {
		intakeSolenoid.set(open);
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
		retVal += "ElevatorFault:" + (isElevatorFaulted() || mElevatorControl == ElevatorControl.OFF) + ";";
		retVal += "ElevatorHomeSwitch:" + mElevatorHomeSwitch.get() + ";";
		retVal += "ElevatorMode:" + mElevatorControl.toString() + ";";

		retVal += "Arm1PosReq:" + armRotation + ";";
		retVal += "Arm1PosAct:" + (mArmMotor.getSelectedSensorPosition(0) / Constants.kSensorUnitsPerRotation / Constants.kArmEncoderGearRatio / Constants.kArmFinalRotationsPerDegree) + ";";
		retVal += "ArmFault:" + (isArmFaulted() || mArmControl == ArmControl.OPEN_LOOP) + ";";
		retVal += "ArmMode:" + mArmControl.toString() + ";";

		retVal += "HasCube:" + mCubeSensor.get() + ";";

		retVal += "Intake1Current:" + mIntakeMotor.getOutputCurrent() + ";";
		retVal += "Intake2Current:" + mIntake2Motor.getOutputCurrent() + ";";
		//TODO: Monitor performance of this method
		retVal += "ScaleHeightInches:" + AutoSelectionReceiver.getInstance().getScaleHeightInches() + ";";

		return retVal;
	}


	@Override
	public boolean runDiagnostics() {
		if (ds.isTest() && Constants.ENABLE_CUBE_HANDLER_DIAG) {
			ConsoleReporter.report("Testing CubeHandler---------------------------------");
			boolean testPassed = true;
			//testPassed &= runArmDiagnostics();
			testPassed &= runElevatorDiagnostics();
			//testPassed &= runIntakeDiagnostics();
			return testPassed;
		} else
			return true;
	}

	private boolean runArmDiagnostics() {
		ConsoleReporter.report("Testing Arm---------------------------------");
		final double kLowCurrentThres = Constants.kArmTestLowCurrentThresh;
		final double kLowRpmThres = Constants.kArmTestLowRPMThresh;

		ArrayList<MotorDiagnostics> mArmDiagArr = new ArrayList<MotorDiagnostics>();
		mArmDiagArr.add(new MotorDiagnostics("Arm Joint", mArmMotor, Constants.kArmTestSpeed, Constants.kArmTestDuration, false));

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

			if (!mD.isSensorInPhase()) {
				ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " Sensor Out of Phase !!!!!!!!!!");
				failure = true;
			}

		}

		return !failure;
	}

	private boolean runElevatorDiagnostics() {
		ConsoleReporter.report("Testing Elevator---------------------------------");
		final double kLowCurrentThres = Constants.kElevatorTestLowCurrentThresh;
		final double kLowRpmThres = Constants.kElevatorTestLowRPMThresh;

		ArrayList<MotorDiagnostics> mElevatorDiagArr = new ArrayList<MotorDiagnostics>();
		mElevatorDiagArr.add(new MotorDiagnostics("Elevator Motor Master", mElevatorMotorMaster, Constants.kElevatorTestSpeed, Constants.kElevatorTestDuration, false));
		mElevatorDiagArr.add(new MotorDiagnostics("Elevator Motor Slave 1", mElevatorMotorSlave, mElevatorMotorMaster, Constants.kElevatorTestSpeed, Constants.kElevatorTestDuration, false));
		mElevatorDiagArr.add(new MotorDiagnostics("Elevator Motor Slave 2", mElevatorMotorSlave2,  mElevatorMotorMaster, Constants.kElevatorTestSpeed, Constants.kElevatorTestDuration, false));
		mElevatorDiagArr.add(new MotorDiagnostics("Elevator Motor Slave 3", mElevatorMotorSlave3, mElevatorMotorMaster, Constants.kElevatorTestSpeed, Constants.kElevatorTestDuration, false));

		boolean failure = false;

		for (MotorDiagnostics mD : mElevatorDiagArr) {
			mD.setZero();
		}

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

			if (!mD.isSensorInPhase()) {
				ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " Sensor Out of Phase !!!!!!!!!!");
				failure = true;
			}

		}

		if (mElevatorDiagArr.size() > 0) {
			List<Double> elevatorMotorCurrents = mElevatorDiagArr.stream().map(MotorDiagnostics::getMotorCurrent).collect(Collectors.toList());
			if (!Util.allCloseTo(elevatorMotorCurrents, elevatorMotorCurrents.get(0), Constants.kElevatorTestCurrentDelta)) {
				failure = true;
				ConsoleReporter.report("!!!!!!!!!!!!!!!!!! Elevator Motor Currents Different !!!!!!!!!!");
			}

			List<Double> elevatorMotorRPMs = mElevatorDiagArr.stream().map(MotorDiagnostics::getMotorRPM).collect(Collectors.toList());
			if (!Util.allCloseTo(elevatorMotorRPMs, elevatorMotorRPMs.get(0), Constants.kElevatorTestRPMDelta)) {
				failure = true;
				ConsoleReporter.report("!!!!!!!!!!!!!!!!!!! Elevator RPMs different !!!!!!!!!!!!!!!!!!!");
			}
		} else {
			ConsoleReporter.report("Elevator Testing Error Occurred in system. Please check code!", MessageLevel.ERROR);
		}

		return !failure;
	}

	private boolean runIntakeDiagnostics() {
		ConsoleReporter.report("Testing Intake---------------------------------");
		final double kLowCurrentThres = Constants.kIntakeTestLowCurrentThresh;

		ArrayList<MotorDiagnostics> mIntakeDiagArr = new ArrayList<MotorDiagnostics>();
		mIntakeDiagArr.add(new MotorDiagnostics("Intake", mIntakeMotor, Constants.kIntakeTestSpeed, Constants.kIntakeTestDuration, false));
		mIntakeDiagArr.add(new MotorDiagnostics("Intake 2", mIntake2Motor, Constants.kIntakeTestSpeed, Constants.kIntakeTestDuration, false));

		boolean failure = false;

		for (MotorDiagnostics mD : mIntakeDiagArr) {
			mD.runTest();

			if (mD.isCurrentUnderThreshold(kLowCurrentThres)) {
				ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " Current Low !!!!!!!!!!");
				failure = true;
			}
		}

		if (mIntakeDiagArr.size() > 0) {
			List<Double> intakeMotorCurrents = mIntakeDiagArr.stream().map(MotorDiagnostics::getMotorCurrent).collect(Collectors.toList());
			if (!Util.allCloseTo(intakeMotorCurrents, intakeMotorCurrents.get(0), Constants.kElevatorTestCurrentDelta)) {
				failure = true;
				ConsoleReporter.report("!!!!!!!!!!!!!!!!!! Intake Motor Currents Different !!!!!!!!!!");
			}

		} else {
			ConsoleReporter.report("Intake Testing Error Occurred in system. Please check code!", MessageLevel.ERROR);
		}

		return !failure;
	}

	private synchronized boolean checkIfArmIsFaulted() {
		//Remove when production
//		if (mArmControl != ArmControl.POSITION)
//			return false;
		//ENDTODO

		boolean armSensorPresent = mArmMotor.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;
		//armSensorPresent = true;

		ErrorCode errorCode = mArmMotor.getLastError();
		if (errorCode != ErrorCode.OK) {
			ConsoleReporter.report("Error getting Arm encoder pulse due to talon error: " + errorCode.toString(), MessageLevel.ERROR);
			ConsoleReporter.report("Disregarding iteration!", MessageLevel.ERROR);
			armSensorPresent = true;
		}

		if (!armSensorPresent) {
			//Check for 3 consecutive misses of encoder before disabling arm
			if (armEncoderLossCounter++ >= 3) {
				setArmControl(ArmControl.OPEN_LOOP);

				String msg = "Could not detect encoder! \r\n\tArm Encoder Detected: " + armSensorPresent;
				ConsoleReporter.report(msg, MessageLevel.DEFCON1);
				DriverStation.reportError(msg, false);
			}
		} else {
			armEncoderLossCounter = 0;
		}

		setArmFaulted(!armSensorPresent, true);
		//armFault = !armSensorPresent;

		if (mArmMotor.hasResetOccurred()) {
			//setArmControl(ArmControl.OPEN_LOOP);

			if(armSensorPresent) {
				zeroArm((int) mPrevArmEncoderVal);
			}
			else {
				setArmControl(ArmControl.OPEN_LOOP);
			}

			ConsoleReporter.report("Arm Talon has reset! Arm may require rehoming! We tried to prevent this for you, but who knows...", MessageLevel.DEFCON1);
			DashboardReporter.addDiagnosticMessage("ArmResetHasOccurred");

			boolean setSucceeded;
			int retryCounter = 0;

			do {
				setSucceeded = true;
				setSucceeded &= mArmMotor.clearStickyFaults(Constants.kTimeoutMsFast) == ErrorCode.OK;
			} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

			if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
				ConsoleReporter.report("Failed to clear Arm Reset !!!!!!", MessageLevel.DEFCON1);

			setArmFaulted(true, false);
			//armFault = true;
		}
		else if (armSensorPresent){
			mPrevArmEncoderVal = mArmMotor.getSelectedSensorPosition(0);
		}

		//armFault = false;

		//TEMP FOR REMOTE SENSOR
		//return false;

		return armFault;
	}

	private synchronized boolean checkIfIntakeIsFaulted() {

		if (mIntakeMotor.hasResetOccurred()) {

			ConsoleReporter.report("Intake 1 Talon has reset!", MessageLevel.DEFCON1);
			DashboardReporter.addDiagnosticMessage("Intake1ResetHasOccurred");

			boolean setSucceeded;
			int retryCounter = 0;

			do {
				setSucceeded = true;
				setSucceeded &= mIntakeMotor.clearStickyFaults(Constants.kTimeoutMsFast) == ErrorCode.OK;
			} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

			if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
				ConsoleReporter.report("Failed to clear Intake 1 Reset !!!!!!", MessageLevel.DEFCON1);

			intakeFault = true;
		}

		if (mIntake2Motor.hasResetOccurred()) {

			ConsoleReporter.report("Intake 2 Talon has reset!", MessageLevel.DEFCON1);
			DashboardReporter.addDiagnosticMessage("Intake2ResetHasOccurred");

			boolean setSucceeded;
			int retryCounter = 0;

			do {
				setSucceeded = true;
				setSucceeded &= mIntake2Motor.clearStickyFaults(Constants.kTimeoutMsFast) == ErrorCode.OK;
			} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

			if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
				ConsoleReporter.report("Failed to clear Intake 2 Reset !!!!!!", MessageLevel.DEFCON1);

			intakeFault = true;
		}


		return intakeFault;
	}

	public boolean isArmFaulted() {
		return armFault;
	}

	private synchronized void setArmFaulted(boolean fault, boolean or) {
		armFault = or ? armFault | fault : fault;
	}

	private synchronized void setElevatorFaulted(boolean fault, boolean or) {
		elevatorFault = or ? elevatorFault | fault : fault;
	}

	private synchronized boolean checkIfElevatorIsFaulted() {
		//Remove when production
//		if (mElevatorControl != ElevatorControl.POSITION)
//			return false;
		//ENDTODO

		boolean elevatorSensorPresent = mElevatorMotorMaster.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;

		if (!elevatorSensorPresent) {
			setElevatorControl(ElevatorControl.OFF);

			String msg = "Could not detect encoder! \r\n\tElevator Encoder Detected: " + elevatorSensorPresent;
			ConsoleReporter.report(msg, MessageLevel.DEFCON1);
			DriverStation.reportError(msg, false);
		}

		setElevatorFaulted(!elevatorSensorPresent, true);
		//elevatorFault = !elevatorSensorPresent;

		if (mElevatorMotorMaster.hasResetOccurred()) {
			setElevatorControl(ElevatorControl.OFF);

			ConsoleReporter.report("Elevator requires rehoming!", MessageLevel.DEFCON1);
			DashboardReporter.addDiagnosticMessage("ElevatorMasterResetHasOccurred");

			boolean setSucceeded;
			int retryCounter = 0;

			do {
				setSucceeded = true;
				setSucceeded &= mElevatorMotorMaster.clearStickyFaults(Constants.kTimeoutMsFast) == ErrorCode.OK;
			} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

			if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
				ConsoleReporter.report("Failed to clear Elevator Reset !!!!!!", MessageLevel.DEFCON1);

			setElevatorFaulted(true, false);
			//elevatorFault = true;
		}

		DashboardReporter.addDiagnosticMessage(TalonHelper.checkMotorReset(mElevatorMotorSlave, "ElevatorSlave1"));
		DashboardReporter.addDiagnosticMessage(TalonHelper.checkMotorReset(mElevatorMotorSlave2, "ElevatorSlave2"));
		DashboardReporter.addDiagnosticMessage(TalonHelper.checkMotorReset(mElevatorMotorSlave3, "ElevatorSlave3"));

		return elevatorFault;
	}


	public boolean isElevatorFaulted() {
		return elevatorFault;
	}

	@Override
	public boolean isSystemFaulted() {
		//Spock logic
		boolean allSensorsPresent = true;

		allSensorsPresent &= !checkIfArmIsFaulted();
		allSensorsPresent &= !checkIfElevatorIsFaulted();
		allSensorsPresent &= !checkElevatorCurrent();
		allSensorsPresent &= !checkIfIntakeIsFaulted();

		return !allSensorsPresent;
	}

	private boolean checkElevatorCurrent() {
		double averageCurrent = getElevatorCurrentAverage();
		boolean retVal = false;

		if (averageCurrent > Constants.kElevatorSafetyCurrent) {
			currentOverageCounter++;
			retVal = true;
		}
		else
			currentOverageCounter = 0;

		if (currentOverageCounter >= 2) {
			currentOverageCounter = 0;
			doCurrentSpikeDetected();
		}

		return retVal;
	}

	public synchronized void setBlinkOnHome(boolean blinkOnHome) {
		this.blinkOnHome = blinkOnHome;
	}

	public synchronized void setElevatorHeight(double elevatorHeight) {
		this.elevatorHeight = Util.limit(elevatorHeight, Constants.kElevatorSoftMin + Constants.kElevatorSafetyDelta, Constants.kElevatorSoftMax - Constants.kElevatorSafetyDelta);
	}

	public synchronized void setArmRotationDeg(double armRotation) {
		this.armRotation = Util.limit(armRotation,Constants.kArmSoftMin / Constants.kArmFinalRotationsPerDegree, Constants.kArmSoftMax / Constants.kArmFinalRotationsPerDegree);
	}

	public boolean isArmAtSetpoint() {
		return Math.abs(getArmRotationDeg() - armRotation) <= Constants.kArmDeviationThresholdDeg;
	}

	public boolean isElevatorAtSetpoint() {
		return Math.abs(getElevatorHeight() - elevatorHeight) <= Constants.kElevatorSetDeviationThreshold;
	}

	public boolean isElevatorPositionControlled() {
		return mElevatorControl == ElevatorControl.POSITION && !requestElevatorHoming;
	}

	public synchronized void incrementElevatorHeight() {
		setElevatorHeight(elevatorHeight + Constants.kElevatorStepSize);
	}

	public synchronized void decrementElevatorHeight() {
		setElevatorHeight(elevatorHeight - Constants.kElevatorStepSize);
	}

	public double getElevatorHeight() {
		return mElevatorMotorMaster.getSelectedSensorPosition(0) / Constants.kSensorUnitsPerRotation / Constants.kElevatorEncoderGearRatio;
	}

	public double getElevatorDesiredHeight() {
		return elevatorHeight;
	}

	public double getArmRotationDeg() {
		return mArmMotor.getSelectedSensorPosition(0) / Constants.kSensorUnitsPerRotation / Constants.kArmEncoderGearRatio / Constants.kArmFinalRotationsPerDegree;
	}

	public double getArmDesiredPosition() {
		return armRotation;
	}

	public double getArmAbsolutePosition() {
		return mArmMotor.getSensorCollection().getPulseWidthPosition();
	}

	public ArmControl getArmControlMode() {
		return mArmControl;
	}

	private void doCurrentSpikeDetected() {
		double currentHeightNative = mElevatorMotorMaster.getSelectedSensorPosition(0);
		double currentHeightRot = QuickMaths.convertNativeUnitsToRotations(currentHeightNative);
		if (mElevatorMotorMaster.getClosedLoopTarget(0) <= currentHeightNative)
			setElevatorHeight(currentHeightRot + 4 * Constants.kElevatorStepSize);
//		else
//			setElevatorHeight(currentHeightRot - 2 * Constants.kElevatorStepSize);
	}

	private double getElevatorCurrentAverage() {
		double elevatorCurrentAverage = mElevatorMotorMaster.getOutputCurrent();
		elevatorCurrentAverage += mElevatorMotorSlave.getOutputCurrent();
		elevatorCurrentAverage += mElevatorMotorSlave2.getOutputCurrent();
		elevatorCurrentAverage += mElevatorMotorSlave3.getOutputCurrent();
		elevatorCurrentAverage /= 4;
		return elevatorCurrentAverage;
	}

	public synchronized void setOpenLoopElevator(double elevatorOpenLoopVal) {
		this.elevatorOpenLoopVal = elevatorOpenLoopVal;
	}

	public synchronized void setDisableFastDown(boolean disable) {
		this.disableFastDown = disable;
	}

	public boolean hasCube() {
		return mCubeSensor.get();
	}

	public synchronized void setDisableCollisionPrevention(boolean disable) {
		this.disableCollisionPrevention = disable;
	}

	public boolean isCollisionPreventionDisabled() {
		return disableCollisionPrevention;
	}

	public synchronized void prepareCLimb() {
		setIntakeClamp(true);
		//setDisableCollisionPrevention(true);
		//setArmRotationDeg(ArmPosition.GET_CLIMBER_HOOK);
	}
}


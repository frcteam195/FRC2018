package org.usfirst.frc.team195.robot.Subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.*;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ArmControl;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ElevatorControl;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ElevatorPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;
import org.usfirst.frc.team195.robot.Utilities.Drivers.KnightDigitalInput;
import org.usfirst.frc.team195.robot.Utilities.Drivers.TalonHelper;
import org.usfirst.frc.team195.robot.Utilities.Drivers.TuneablePID;
import org.usfirst.frc.team195.robot.Utilities.Loops.Loop;
import org.usfirst.frc.team195.robot.Utilities.Loops.Looper;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Util;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class CubeHandlerSubsystem implements CriticalSystemStatus, CustomSubsystem, DiagnosableSubsystem, Reportable {
	private static CubeHandlerSubsystem instance;
	private IntakeControl mIntakeControl;
	private IntakeControl mPrevIntakeControl;
	private ArmControl mArmControl;
	private ArmControl mPrevArmControl;
	private ElevatorControl mElevatorControl;
	private ElevatorControl mPrevElevatorControl;

	private TuneablePID tuneableArmJoint;
	private TuneablePID tuneableElevator;

	private TalonSRX mArmMotor;
	private TalonSRX mIntake2Motor;
	private TalonSRX mIntakeMotor;
	private TalonSRX mElevatorMotorMaster;
	private BaseMotorController mElevatorMotorSlave;
	private BaseMotorController mElevatorMotorSlave2;
	private BaseMotorController mElevatorMotorSlave3;

	private Solenoid intakeSolenoid;

	private DriverStation ds;

	private KnightDigitalInput mElevatorHomeSwitch;

	private boolean elevatorFault = false;
	private boolean armFault = false;

	private double elevatorHeight = 0;
	private double mPrevElevatorHeight = 0;
	private double currentOverageCounter = 0;
	private double homingTimeStart = 0;



	private CubeHandlerSubsystem() throws Exception {
		ds = DriverStation.getInstance();
		Controllers robotControllers = Controllers.getInstance();

		mArmMotor = robotControllers.getArm1Motor();
		mIntakeMotor = robotControllers.getIntakeMotor();
		mIntake2Motor = robotControllers.getIntake2Motor();
		mElevatorMotorMaster = robotControllers.getElevatorMotorMaster();
		mElevatorMotorSlave = robotControllers.getElevatorMotorSlave();
		mElevatorMotorSlave2 = robotControllers.getElevatorMotorSlave2();
		mElevatorMotorSlave3 = robotControllers.getElevatorMotorSlave3();

		mElevatorHomeSwitch = robotControllers.getElevatorHomeSwitch();

		intakeSolenoid = robotControllers.getIntakeSolenoid();

		mIntakeControl = IntakeControl.OFF;
		mPrevIntakeControl = IntakeControl.OFF;

		mArmControl = ArmControl.POSITION;
		mPrevArmControl = ArmControl.OFF;

		mElevatorControl = ElevatorControl.POSITION;
		mPrevElevatorControl = ElevatorControl.OFF;


//		tuneableArmJoint = new TuneablePID("Arm 1 Joint", mArmMotor, null, 5807, true, true);
//		tuneableArmJoint.start();
//		tuneableArmJoint = new TuneablePID("Arm 2 Joint", mIntake2Motor, null, 5808, true, true);
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


		mElevatorMotorMaster.setSensorPhase(false);
		mElevatorMotorMaster.setInverted(true);
		mElevatorMotorSlave3.setInverted(true);
		mElevatorMotorMaster.setNeutralMode(NeutralMode.Brake);
		mElevatorMotorSlave.setNeutralMode(NeutralMode.Brake);
		mElevatorMotorSlave2.setNeutralMode(NeutralMode.Brake);
		mElevatorMotorSlave3.setNeutralMode(NeutralMode.Brake);

		mIntakeMotor.setInverted(false);
		mIntake2Motor.setInverted(false);
		mIntakeMotor.setNeutralMode(NeutralMode.Coast);
		mIntake2Motor.setNeutralMode(NeutralMode.Coast);


		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			setSucceeded &= mArmMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mIntake2Motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mArmMotor.configContinuousCurrentLimit(Constants.kArm1MaxContinuousCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArmMotor.configPeakCurrentLimit(Constants.kArm1MaxPeakCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArmMotor.configPeakCurrentDuration(Constants.kArm1MaxPeakCurrentDurationMS, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mIntakeMotor.configContinuousCurrentLimit(Constants.kIntakeMaxContinuousCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mIntakeMotor.configPeakCurrentLimit(Constants.kIntakeMaxPeakCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mIntakeMotor.configPeakCurrentDuration(Constants.kIntakeMaxPeakCurrentDurationMS, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mIntake2Motor.configContinuousCurrentLimit(Constants.kIntakeMaxContinuousCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mIntake2Motor.configPeakCurrentLimit(Constants.kIntakeMaxPeakCurrentLimit, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mIntake2Motor.configPeakCurrentDuration(Constants.kIntakeMaxPeakCurrentDurationMS, Constants.kTimeoutMs) == ErrorCode.OK;

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

			setSucceeded &= mArmMotor.configForwardSoftLimitThreshold((int) (Constants.kArm1SoftMax * Constants.kArm1EncoderGearRatio * Constants.kSensorUnitsPerRotation), Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArmMotor.configReverseSoftLimitThreshold((int) (Constants.kArm1SoftMin * Constants.kArm1EncoderGearRatio * Constants.kSensorUnitsPerRotation), Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArmMotor.configForwardSoftLimitEnable(true, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArmMotor.configReverseSoftLimitEnable(true, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mArmMotor.configAllowableClosedloopError(0, Constants.kArm1AllowedError, Constants.kTimeoutMs) == ErrorCode.OK;

			setSucceeded &= mElevatorMotorMaster.configForwardSoftLimitThreshold((int) (Constants.kElevatorSoftMax * Constants.kElevatorEncoderGearRatio * Constants.kSensorUnitsPerRotation), Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mElevatorMotorMaster.configReverseSoftLimitThreshold((int) (Constants.kElevatorSoftMin * Constants.kElevatorEncoderGearRatio * Constants.kSensorUnitsPerRotation), Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mElevatorMotorMaster.configForwardSoftLimitEnable(true, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= mElevatorMotorMaster.configReverseSoftLimitEnable(true, Constants.kTimeoutMs) == ErrorCode.OK;

		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		setSucceeded &= TalonHelper.setPIDGains(mArmMotor, 0, Constants.kArm1Kp, Constants.kArm1Ki, Constants.kArm1Kd, Constants.kArm1Kf, Constants.kArm1RampRate, Constants.kArm1IZone);
		setSucceeded &= TalonHelper.setPIDGains(mElevatorMotorMaster, 0, Constants.kElevatorKp, Constants.kElevatorKi, Constants.kElevatorKd, Constants.kElevatorKf, Constants.kElevatorRampRate, Constants.kElevatorIZone);
		setSucceeded &= TalonHelper.setPIDGains(mIntakeMotor, 0, Constants.kIntakeKp, Constants.kIntakeKi, Constants.kIntakeKd, Constants.kIntakeKf, Constants.kIntakeRampRate, Constants.kIntakeIZone);
		setSucceeded &= TalonHelper.setPIDGains(mIntake2Motor,  0, Constants.kIntakeKp, Constants.kIntakeKi, Constants.kIntakeKd, Constants.kIntakeKf, Constants.kIntakeRampRate, Constants.kIntakeIZone);
		setSucceeded &= TalonHelper.setMotionMagicParams(mArmMotor, Constants.kArm1MaxVelocity, Constants.kArm1MaxAccel);
		setSucceeded &= TalonHelper.setMotionMagicParams(mElevatorMotorMaster, Constants.kElevatorMaxVelocity, Constants.kElevatorMaxAccel);

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
		mElevatorMotorMaster.set(ControlMode.Disabled, 0);
		int homeElevatorValue = (int)(Constants.kElevatorSoftMin * Constants.kElevatorEncoderGearRatio * Constants.kSensorUnitsPerRotation);

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
		setElevatorHeight(ElevatorPosition.HOME);

		return retryCounter < Constants.kTalonRetryCount && setSucceeded;
	}

	private boolean zeroArm() {
		mArmMotor.set(ControlMode.Disabled, 0);

		int homeA1Value = (int)(Constants.kArm1SoftMax * Constants.kArm1EncoderGearRatio * Constants.kSensorUnitsPerRotation);

		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			setSucceeded &= mArmMotor.setSelectedSensorPosition(homeA1Value, 0, Constants.kTimeoutMs) == ErrorCode.OK;

		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to zero Arm!!!", MessageLevel.DEFCON1);

		mArmMotor.set(ControlMode.MotionMagic, homeA1Value);

		return retryCounter < Constants.kTalonRetryCount && setSucceeded;
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
				boolean collisionOccurring = DriveBaseSubsystem.getInstance().isEmergencySafetyRequired();
//				SmartDashboard.putBoolean("ElevatorHomeSwitch", mElevatorHomeSwitch.get());
//				SmartDashboard.putString("ElevatorControlMode", mElevatorControl.toString());
				switch (mArmControl) {
					case POSITION:
						if (collisionOccurring) {

						}

						//TODO: Add arm code

						break;
					case MANUAL:
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
						if (collisionOccurring) {
							//setElevatorHeight(ElevatorPosition.HOME);
						}

						if (elevatorHeight <= ElevatorPosition.HOME &&
								Math.abs(QuickMaths.convertNativeUnitsToRotations(mElevatorMotorMaster.getSelectedSensorPosition(0)) - elevatorHeight)
										< Constants.kElevatorDeviationThreshold &&
								mElevatorHomeSwitch.get()) {
							setElevatorControl(ElevatorControl.HOMING);
							break;
						}

						if (mElevatorHomeSwitch.getFallingEdge()) {
							zeroElevator();
						} else if (elevatorHeight != mPrevElevatorHeight) {
							mElevatorMotorMaster.set(ControlMode.MotionMagic, elevatorHeight * Constants.kSensorUnitsPerRotation * Constants.kElevatorEncoderGearRatio);

							mPrevElevatorHeight = elevatorHeight;
						}
						break;
					case MANUAL:
						break;
					case HOMING:
						if (mPrevElevatorControl != ElevatorControl.HOMING)
							homingTimeStart = Timer.getFPGATimestamp();

						mElevatorMotorMaster.configReverseSoftLimitEnable(false, Constants.kTimeoutMsFast);
						mElevatorMotorMaster.set(ControlMode.PercentOutput, Constants.kElevatorHomingSpeed);
//						mElevatorMotorSlave.set(ControlMode.PercentOutput, Constants.kElevatorHomingSpeed);
//						mElevatorMotorSlave2.set(ControlMode.PercentOutput, Constants.kElevatorHomingSpeed);
//						mElevatorMotorSlave3.set(ControlMode.PercentOutput, Constants.kElevatorHomingSpeed);
						if (!mElevatorHomeSwitch.get()) {
//							mElevatorMotorSlave.set(ControlMode.Disabled, 0);
//							mElevatorMotorSlave2.set(ControlMode.Disabled, 0);
//							mElevatorMotorSlave3.set(ControlMode.Disabled, 0);
							zeroElevator();
//							mElevatorMotorSlave.follow(mElevatorMotorMaster);
//							mElevatorMotorSlave2.follow(mElevatorMotorMaster);
//							mElevatorMotorSlave3.follow(mElevatorMotorMaster);
							setElevatorControl(ElevatorControl.POSITION);
						}

						if (Timer.getFPGATimestamp() - homingTimeStart > Constants.kElevatorHomingTimeout) {
							setElevatorControl(ElevatorControl.OFF);
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


				if (mIntakeControl != mPrevIntakeControl) {
					switch (mIntakeControl) {
						case INTAKE_IN:
//							mIntakeMotor.set(ControlMode.Current, 25);
//							mIntake2Motor.set(ControlMode.Current, 25);
							mIntakeMotor.set(ControlMode.PercentOutput, 1);
							mIntake2Motor.set(ControlMode.PercentOutput, 1);
							break;
						case INTAKE_OUT:
//							mIntakeMotor.set(ControlMode.Current, -55);
//							mIntake2Motor.set(ControlMode.Current, -55);
							mIntakeMotor.set(ControlMode.PercentOutput, -1);
							mIntake2Motor.set(ControlMode.PercentOutput, -1);
							break;
						case INTAKE_OUT_HALFSPEED:
//							mIntakeMotor.set(ControlMode.Current, -55);
//							mIntake2Motor.set(ControlMode.Current, -55);
							mIntakeMotor.set(ControlMode.PercentOutput, -0.5);
							mIntake2Motor.set(ControlMode.PercentOutput, -0.5);
							break;
						case OFF:
						default:
							if (mIntakeMotor.getControlMode() != ControlMode.Disabled || mIntake2Motor.getControlMode() != ControlMode.Disabled) {
								mIntakeMotor.set(ControlMode.Disabled, 0);
								mIntake2Motor.set(ControlMode.Disabled, 0);
							}
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
		retVal += "ElevatorFault:" + isElevatorFaulted() + ";";

		retVal += "Arm1PosReq:" + "" + ";";
		retVal += "Arm1PosAct:" + mArmMotor.getSelectedSensorPosition(0) + ";";
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
		mArmDiagArr.add(new MotorDiagnostics("Arm Joint 1", mArmMotor, Constants.kArm1TestSpeed, Constants.kArm1TestDuration, true));

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

		if (mArmDiagArr.size() > 0) {
			List<Double> armMotorCurrents = mArmDiagArr.stream().map(MotorDiagnostics::getMotorCurrent).collect(Collectors.toList());
			if (!Util.allCloseTo(armMotorCurrents, armMotorCurrents.get(0), Constants.kArmTestCurrentDelta)) {
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
		boolean allSensorsPresent = true;

		boolean arm1SensorPresent = mArmMotor.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;
		allSensorsPresent &= arm1SensorPresent;

		if (!allSensorsPresent) {
			setArmControl(ArmControl.OFF);

			String msg = "Could not detect encoder! \r\n\tArm 1 Encoder Detected: " + arm1SensorPresent;
			ConsoleReporter.report(msg, MessageLevel.DEFCON1);
			DriverStation.reportError(msg, false);
		}
		armFault = !allSensorsPresent;

		if (mArmMotor.hasResetOccurred()) {
			setElevatorControl(ElevatorControl.OFF);

			ConsoleReporter.report("Arm requires rehoming!", MessageLevel.DEFCON1);

			boolean setSucceeded;
			int retryCounter = 0;

			do {
				setSucceeded = true;
				setSucceeded &= mArmMotor.clearStickyFaults(Constants.kTimeoutMsFast) == ErrorCode.OK;
			} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

			if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
				ConsoleReporter.report("Failed to clear Arm Reset !!!!!!", MessageLevel.DEFCON1);

			armFault = true;
		}

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

		if (mElevatorMotorMaster.hasResetOccurred()) {
			setElevatorControl(ElevatorControl.OFF);

			ConsoleReporter.report("Elevator requires rehoming!", MessageLevel.DEFCON1);

			boolean setSucceeded;
			int retryCounter = 0;

			do {
				setSucceeded = true;
				setSucceeded &= mElevatorMotorMaster.clearStickyFaults(Constants.kTimeoutMsFast) == ErrorCode.OK;
			} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

			if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
				ConsoleReporter.report("Failed to clear Elevator Reset !!!!!!", MessageLevel.DEFCON1);

			elevatorFault = true;
		}

		return elevatorFault;
	}

	public boolean isElevatorFaulted() {
		return elevatorFault;
	}

	@Override
	public boolean isSystemFaulted() {
		//Spock logic
		boolean allSensorsPresent = true;

		//allSensorsPresent &= !checkIfArmIsFaulted();
		allSensorsPresent &= !checkIfElevatorIsFaulted();
		allSensorsPresent &= !checkElevatorCurrent();

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

	public synchronized void setElevatorHeight(double elevatorHeight) {
		this.elevatorHeight = Util.limit(elevatorHeight, Constants.kElevatorSoftMin + Constants.kElevatorSafetyDelta, Constants.kElevatorSoftMax - Constants.kElevatorSafetyDelta);
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
}


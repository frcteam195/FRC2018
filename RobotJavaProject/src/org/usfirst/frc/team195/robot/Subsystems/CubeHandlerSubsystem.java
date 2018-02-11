package org.usfirst.frc.team195.robot.Subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.*;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.Arm.ArmConfiguration;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.Arm.PointFinder;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.Arm.PolarCoordinate;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ArmControl;
import org.usfirst.frc.team195.robot.Utilities.Drivers.TalonHelper;
import org.usfirst.frc.team195.robot.Utilities.Drivers.TuneablePID;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import org.usfirst.frc.team195.robot.Utilities.Loops.Loop;
import org.usfirst.frc.team195.robot.Utilities.Loops.Looper;

public class CubeHandlerSubsystem implements CriticalSystemStatus, CustomSubsystem, DiagnosableSubsystem, Reportable {
	
	private static final int MIN_CUBE_HANDLER_THREAD_LOOP_TIME_MS = 20;
	private static CubeHandlerSubsystem instance;
	private IntakeControl mIntakeControl;
	private IntakeControl mPrevIntakeControl;
	private ArmControl mArmControl;
	private ArmControl mPrevArmControl;

	private TuneablePID tuneableArmJoint;

	private TalonSRX arm1Motor;
	private TalonSRX arm2Motor;
	
	private Solenoid ginoSol;

	private DriverStation ds;

	private double requestedElevatorPos;

	private ThreadRateControl threadRateControl = new ThreadRateControl();

	private ArmConfiguration armConfiguration = null;
	private PointFinder pointFinder;


	private CubeHandlerSubsystem() throws Exception {
		super();
		ds = DriverStation.getInstance();
		Controllers robotControllers = Controllers.getInstance();

		arm1Motor = robotControllers.getArm1Motor();
		arm2Motor = robotControllers.getArm2Motor();

		ginoSol = robotControllers.getGinoSol();

		requestedElevatorPos = 0;

		mIntakeControl = IntakeControl.OFF;
		mPrevIntakeControl = IntakeControl.OFF;

		mArmControl = ArmControl.POSITION;
		mPrevArmControl = ArmControl.OFF;

		pointFinder = new PointFinder();
		pointFinder.setA1AngleRange(0, 180);
		pointFinder.setA2AngleRange(-160, 160);

		arm1Motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);
		arm2Motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.kTimeoutMs);

		arm1Motor.setInverted(true);
		arm1Motor.setSensorPhase(true);

		arm2Motor.setInverted(true);

		arm1Motor.configContinuousCurrentLimit(35, Constants.kTimeoutMs);
		arm1Motor.configPeakCurrentLimit(45, Constants.kTimeoutMs);
		arm1Motor.configPeakCurrentDuration(400, Constants.kTimeoutMs);
		arm1Motor.enableCurrentLimit(true);
		arm2Motor.configContinuousCurrentLimit(35, Constants.kTimeoutMs);
		arm2Motor.configPeakCurrentLimit(45, Constants.kTimeoutMs);
		arm2Motor.configPeakCurrentDuration(400, Constants.kTimeoutMs);
		arm2Motor.enableCurrentLimit(true);


		TalonHelper.setPIDGains(arm1Motor, 0, 1, 0.006, 4, 0.8, 0, 10);
		TalonHelper.setPIDGains(arm2Motor, 0, 1, 0.006, 4, 0.8, 0, 10);
		TalonHelper.setMotionMagicParams(arm1Motor, 300, 250);
//		TalonHelper.setMotionMagicParams(arm1Motor, 300, 550);
//		TalonHelper.setMotionMagicParams(arm2Motor, 700, 950);
		TalonHelper.setMotionMagicParams(arm2Motor, 700, 450);

		//tuneableArmJoint = new TuneablePID("Arm 1 Joint", arm1Motor, null, 5807, true, true);
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
//		intakeMotor2.setInverted(true);
//
//		TalonHelper.setPIDGains(intakeMotor1, 0, 0.2, 0, 0, 0.06);
//		TalonHelper.setPIDGains(intakeMotor2, 0, 0.2, 0, 0, 0.06);
//		intakeMotor1.set(ControlMode.Current, 0);
//		intakeMotor2.set(ControlMode.Current, 0);

		isSystemFaulted();
	}

	@Override
	public void subsystemHome() {
		arm1Motor.set(ControlMode.Disabled, 0);
		arm2Motor.set(ControlMode.Disabled, 0);
		int homeA1Value = (int)(5.0 * Constants.kSensorUnitsPerRotation);
		int homeA2Value = (int)(-4.56348 * Constants.kSensorUnitsPerRotation);

		arm1Motor.setSelectedSensorPosition(homeA1Value, 0, Constants.kTimeoutMs);
		arm1Motor.setSelectedSensorPosition(homeA1Value, 0, Constants.kTimeoutMs);
		arm2Motor.setSelectedSensorPosition(homeA2Value, 0, Constants.kTimeoutMs); //max pos 4.56226, min pos -4.56348
		arm2Motor.setSelectedSensorPosition(homeA2Value, 0, Constants.kTimeoutMs); //max pos 4.56226, min pos -4.56348
		try {
			Thread.sleep(100);
		} catch (Exception ex) {
			ConsoleReporter.report(ex);
		}
		arm1Motor.set(ControlMode.MotionMagic, homeA1Value);
		arm2Motor.set(ControlMode.MotionMagic, homeA2Value);
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
//				intakeMotor1.set(ControlMode.Current, 0);
//				intakeMotor2.set(ControlMode.Current, 0);
			}
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized (CubeHandlerSubsystem.this) {
					switch (mArmControl) {
						case POSITION:
							if (armConfiguration != null) {
								//ConsoleReporter.report("" + convertAngleToSRX(armConfiguration.getA1Angle(), Constants.kSensorUnitsPerRotation * Constants.kArm1EncoderGearRatio));
								//ConsoleReporter.report("" + convertAngleToSRX(armConfiguration.getA2Angle(), Constants.kSensorUnitsPerRotation * Constants.kArm2EncoderGearRatio));
								arm1Motor.set(ControlMode.MotionMagic, convertAngleToSRX(armConfiguration.getA1Angle(), Constants.kSensorUnitsPerRotation * Constants.kArm1EncoderGearRatio));
								arm2Motor.set(ControlMode.MotionMagic, convertAngleToSRX(armConfiguration.getA2Angle(), Constants.kSensorUnitsPerRotation * Constants.kArm2EncoderGearRatio));
							}
							break;
						case MANUAL:
							break;
						case OFF:
						default:
							arm1Motor.set(ControlMode.Disabled, 0);
							arm2Motor.set(ControlMode.Disabled, 0);
							break;
					}
					mPrevArmControl = mArmControl;

				if (mIntakeControl != mPrevIntakeControl) {
					switch (mIntakeControl) {
						case FORWARD:
//							intakeMotor1.set(ControlMode.Current, 25);
//							intakeMotor2.set(ControlMode.Current, 25);
							break;
						case REVERSE:
//							intakeMotor1.set(ControlMode.Current, -35);
//							intakeMotor2.set(ControlMode.Current, -35);
							break;
						case OFF:
						default:
//							intakeMotor1.set(ControlMode.Current, 0);
//							intakeMotor2.set(ControlMode.Current, 0);
							break;
					}
					mPrevIntakeControl = mIntakeControl;
				}
			}
		}
		@Override
		public void onStop(double timestamp) {
//			intakeMotor1.set(ControlMode.Current, 0);
//			intakeMotor2.set(ControlMode.Current, 0);
		}
	};

	@Override
	public void registerEnabledLoops(Looper in) {
		in.register(mLoop);
	}


	@Override
	public void terminate() {
		ConsoleReporter.report("CAN'T STOP, WON'T STOP, DON'T CALL ME!", MessageLevel.ERROR);
//		runThread = false;
//		try {
//			super.join(Constants.kThreadJoinTimeout);
//		} catch (Exception ex) {
//			ConsoleReporter.report(ex);
//		}
	}

	private double convertAngleToSRX(double angle, double countsPerRev) {
		return angle * countsPerRev / 360;
	}

	public synchronized void setArmCoordinate(PolarCoordinate polarCoordinate) {
		armConfiguration = pointFinder.getArmConfigFromPolar(polarCoordinate);
//		if (armConfiguration != null) {
//			ConsoleReporter.report("Got my point!");
//			ConsoleReporter.report(armConfiguration.toString());
//		}
	}
	
	public synchronized void setIntakeOpen(boolean open) {
		ginoSol.set(open);
	}
	
	public boolean isElevatorFaulted() {
		return false;
	}
	
	public double getElevatorPos() {
		return requestedElevatorPos;
	}
	
	public synchronized void setIntakeControl(IntakeControl mIntakeControl) {
		this.mIntakeControl = mIntakeControl;
	}

	public synchronized void setArmControl(ArmControl mArmControl) {
		this.mArmControl = mArmControl;
	}

	@Override
	public String toString() {
		return generateReport();
	}

	@Override
	public String generateReport() {
		String retVal = "";
//		retVal += "ElevatorPosReq:" + requestedElevatorPos + ";";
//		retVal += "ElevatorPosAct:" + liftMotor.getSelectedSensorPosition(0) + ";";
//		retVal += "ElevatorFault:" + isElevatorFaulted() + ";";
		return retVal;
	}

	@Override
	public boolean runDiagnostics() {
		arm1Motor.set(ControlMode.PercentOutput, 0.2);
		Timer.delay(0.5);
		arm1Motor.set(ControlMode.PercentOutput, 0.0);
//		ConsoleReporter.report("Testing CubeHandler---------------------------------");
//		final double kLowCurrentThres = 0.5;
//		final double kLowRpmThres = 300;
//
//		ArrayList<MotorDiagnostics> mIntakeDiagArr = new ArrayList<MotorDiagnostics>();
//		mIntakeDiagArr.add(new MotorDiagnostics("Intake Left", intakeMotor1));
//		mIntakeDiagArr.add(new MotorDiagnostics("Intake Right", intakeMotor2));
//
//		boolean failure = false;
//
//		for (MotorDiagnostics mD: mIntakeDiagArr) {
//			mD.runTest();
//
//			if (mD.isCurrentUnderThreshold(kLowCurrentThres)) {
//				ConsoleReporter.report("!!!!!!!!!!!!!!!!!! " + mD.getMotorName() + " Current Low !!!!!!!!!!");
//				failure = true;
//			}
//
//		}
//
//		if (mIntakeDiagArr.size() > 0) {
//			List<Double> intakeMotorCurrents = mIntakeDiagArr.stream().map(u -> u.getMotorCurrent()).collect(Collectors.toList());
//			if (!Util.allCloseTo(intakeMotorCurrents, intakeMotorCurrents.get(0), 5.0)) {
//				failure = true;
//				ConsoleReporter.report("!!!!!!!!!!!!!!!!!! Intake Motor Currents Different !!!!!!!!!!");
//			}
//
//		} else {
//			ConsoleReporter.report("Intake Testing Error Occurred in system. Please check code!", MessageLevel.ERROR);
//		}
//
//		return !failure;

		return true;
	}

	@Override
	public boolean isSystemFaulted() {
		boolean allSensorsPresent = true;

		boolean arm1SensorPresent = arm1Motor.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;
		boolean arm2SensorPresent = arm2Motor.getSensorCollection().getPulseWidthRiseToRiseUs() != 0;
		allSensorsPresent &= arm1SensorPresent;
		allSensorsPresent &= arm2SensorPresent;
		if (!arm1SensorPresent || !arm2SensorPresent) {
			String msg = "Could not detect encoder! \r\n\tArm 1 Encoder Detected: " + arm1SensorPresent + "\r\n\tArm 2 Encoder Detected: " + arm2SensorPresent;
			ConsoleReporter.report(msg, MessageLevel.DEFCON1);
			DriverStation.reportError(msg, false);
		}

		if (!allSensorsPresent)
			setArmControl(ArmControl.OFF);

		return !allSensorsPresent;
	}
}


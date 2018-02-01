package org.usfirst.frc.team195.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.*;
import org.usfirst.frc.team195.robot.Utilities.Drivers.TalonHelper;
import org.usfirst.frc.team195.robot.Utilities.Drivers.TuneablePID;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Util;

public class CubeHandlerSubsystem extends Thread implements CustomSubsystem, DiagnosableSubsystem, Reportable {
	
	private static final int MIN_CUBE_HANDLER_THREAD_LOOP_TIME_MS = 20;
	private static CubeHandlerSubsystem instance;
	private TuneablePID tuneableIntake;
	private IntakeControl intakeControl;

	private TalonSRX liftMotor;
	private VictorSPX liftMotorSlave;
	private TalonSRX intakeMotor1;
	private TalonSRX intakeMotor2;
	private TalonSRX intakeShoulderMotor;
	private TalonSRX intakeElbowMotor;
	
	private Solenoid ginoSol;

	private DriverStation ds;
	
	private boolean runThread;

	private double requestedElevatorPos;

	private ThreadRateControl threadRateControl = new ThreadRateControl();


	private CubeHandlerSubsystem() throws Exception {
		super();
		ds = DriverStation.getInstance();
		Controllers robotControllers = Controllers.getInstance();

		liftMotor = robotControllers.getLiftMotor();
		liftMotorSlave = robotControllers.getLiftMotorSlave();
		intakeMotor1 = robotControllers.getIntakeMotor();
		intakeMotor2 = robotControllers.getIntakeMotor2();
		intakeShoulderMotor = robotControllers.getIntakeShoulderMotor();
		intakeElbowMotor = robotControllers.getIntakeElbowMotor();

		ginoSol = robotControllers.getGinoSol();

		requestedElevatorPos = 0;

		runThread = false;

		intakeControl = IntakeControl.OFF;
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
		intakeMotor2.setInverted(true);
		
		TalonHelper.setPIDGains(intakeMotor1, 0, 0.2, 0, 0, 0.06);
		TalonHelper.setPIDGains(intakeMotor2, 0, 0.2, 0, 0, 0.06);
		intakeMotor1.set(ControlMode.Current, 0);
		intakeMotor2.set(ControlMode.Current, 0);
	}

	@Override
	public void subsystemHome() {
		;
	}

	@Override
	public void start() {
		runThread = true;
		if (!super.isAlive())
			super.start();
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
	
	@Override
	public void run() {
		while (!ds.isEnabled()) {try{Thread.sleep(20);}catch(Exception ex) {}}
		subsystemHome();
		threadRateControl.start();

		while(runThread) {

			switch(intakeControl) {
				case FORWARD:
					intakeMotor1.set(ControlMode.Current, 25);
					intakeMotor2.set(ControlMode.Current, 25);
					break;
				case REVERSE:
					intakeMotor1.set(ControlMode.Current, -35);
					intakeMotor2.set(ControlMode.Current, -35);
					break;
				case OFF:
				default:
					intakeMotor1.set(ControlMode.Current, 0);
					intakeMotor2.set(ControlMode.Current, 0);
					break;
			}

			threadRateControl.doRateControl(MIN_CUBE_HANDLER_THREAD_LOOP_TIME_MS);
		}
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
	
	public synchronized void setIntakeControl(IntakeControl intakeControl) {
		this.intakeControl = intakeControl;
	}
	
	@Override
	public String toString() {
		return generateReport();
	}

	@Override
	public String generateReport() {
		String retVal = "";
		retVal += "ElevatorPosReq:" + requestedElevatorPos + ";";
		retVal += "ElevatorPosAct:" + liftMotor.getSelectedSensorPosition(0) + ";";
		retVal += "ElevatorFault:" + isElevatorFaulted() + ";";
		return retVal;
	}

	@Override
	public boolean runDiagnostics() {
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
}


package org.usfirst.frc.team195.robot.Subsystems;

import java.util.List;

import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.Constants;
import org.usfirst.frc.team195.robot.Utilities.Controllers;
import org.usfirst.frc.team195.robot.Utilities.CustomSubsystem;
import org.usfirst.frc.team195.robot.Utilities.Reportable;
import org.usfirst.frc.team195.robot.Utilities.TalonHelper;
import org.usfirst.frc.team195.robot.Utilities.TuneablePID;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class CubeHandlerSubsystem extends Thread implements CustomSubsystem, Reportable {
	
	private static final int MIN_CUBE_HANDLER_THREAD_LOOP_TIME_MS = 20;
	
	private TuneablePID tuneableIntake;
	
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
		
		tuneableIntake = new TuneablePID("Intake", intakeMotor2, 0, 5807, false, false);
	}
	
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
	
	private double cubeHandlerThreadControlStart, cubeHandlerThreadControlEnd;
	private int cubeHandlerThreadControlElapsedTimeMS;

	private double requestedElevatorPos;

	private static CubeHandlerSubsystem instance;
	
	public static CubeHandlerSubsystem getInstance() {
		if(instance == null) {
			try {
				instance = new CubeHandlerSubsystem();
			} catch (Exception ex) {
				ConsoleReporter.report(ex.toString(), MessageLevel.DEFCON1);
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
		super.start();
		tuneableIntake.start();
	}

	public void terminate() {
		runThread = false;
	}
	
	@Override
	public void run() {
		while (!ds.isEnabled()) {try{Thread.sleep(20);}catch(Exception ex) {}}
		subsystemHome();

		while(runThread) {
			cubeHandlerThreadControlStart = Timer.getFPGATimestamp();
			
			
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
			
			
			do {
				cubeHandlerThreadControlEnd = Timer.getFPGATimestamp();
				cubeHandlerThreadControlElapsedTimeMS = (int) ((cubeHandlerThreadControlEnd - cubeHandlerThreadControlStart) * 1000);
				if (cubeHandlerThreadControlElapsedTimeMS < MIN_CUBE_HANDLER_THREAD_LOOP_TIME_MS)
					try{Thread.sleep(MIN_CUBE_HANDLER_THREAD_LOOP_TIME_MS - cubeHandlerThreadControlElapsedTimeMS);}catch(Exception ex) {};
			} while(cubeHandlerThreadControlElapsedTimeMS < MIN_CUBE_HANDLER_THREAD_LOOP_TIME_MS);
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
	
}


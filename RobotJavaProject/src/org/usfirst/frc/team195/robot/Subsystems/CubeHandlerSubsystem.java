package org.usfirst.frc.team195.robot.Subsystems;

import java.util.List;

import org.usfirst.frc.team195.robot.Utilities.Constants;
import org.usfirst.frc.team195.robot.Utilities.Controllers;
import org.usfirst.frc.team195.robot.Utilities.CustomSubsystem;
import org.usfirst.frc.team195.robot.Utilities.Reportable;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class CubeHandlerSubsystem extends Thread implements CustomSubsystem, Reportable {
	private static final int MIN_CUBE_HANDLER_THREAD_LOOP_TIME_MS = 20;
	
	private CubeHandlerSubsystem() {
		super();
		ds = DriverStation.getInstance();
		Controllers robotControllers = Controllers.getInstance();

		liftMotor = robotControllers.getLiftMotor();
		liftMotorSlave = robotControllers.getLiftMotorSlave();
		intakeMotor = robotControllers.getIntakeMotor();
		intakeMotorSlave = robotControllers.getIntakeMotorSlave();
		intakeActuatorMotor = robotControllers.getIntakeActuatorMotor();
		intakeRotationMotor = robotControllers.getIntakeRotationMotor();

		requestedElevatorPos = 0;
		
		runThread = false;
	}

	private TalonSRX liftMotor;
	private VictorSPX liftMotorSlave;
	private TalonSRX intakeMotor;
	private VictorSPX intakeMotorSlave;
	private TalonSRX intakeActuatorMotor;
	private TalonSRX intakeRotationMotor;

	private DriverStation ds;
	
	private boolean runThread;
	
	private double cubeHandlerThreadControlStart, cubeHandlerThreadControlEnd;
	private int cubeHandlerThreadControlElapsedTimeMS;

	private double requestedElevatorPos;

	private static CubeHandlerSubsystem instance;
	
	public static CubeHandlerSubsystem getInstance() {
		if(instance == null)
			instance = new CubeHandlerSubsystem();
		
		return instance;
	}
	
	public static CubeHandlerSubsystem getInstance(List<CustomSubsystem> subsystemList) {
		subsystemList.add(getInstance());
		return instance;
	}

	@Override
	public void init() {
		
		try {Thread.sleep(20);} catch (Exception ex) {}
	}

	@Override
	public void subsystemHome() {
		
		try {Thread.sleep(20);} catch (Exception ex) {}
	}

	@Override
	public void start() {
		runThread = true;
		super.start();
	}
	
	@Override
	public void run() {
		while (!ds.isEnabled()) {try{Thread.sleep(20);}catch(Exception ex) {}}
		subsystemHome();

		while(runThread) {
			cubeHandlerThreadControlStart = Timer.getFPGATimestamp();

			do {
				cubeHandlerThreadControlEnd = Timer.getFPGATimestamp();
				cubeHandlerThreadControlElapsedTimeMS = (int) ((cubeHandlerThreadControlEnd - cubeHandlerThreadControlStart) * 1000);
				if (cubeHandlerThreadControlElapsedTimeMS < MIN_CUBE_HANDLER_THREAD_LOOP_TIME_MS)
					try{Thread.sleep(MIN_CUBE_HANDLER_THREAD_LOOP_TIME_MS - cubeHandlerThreadControlElapsedTimeMS);}catch(Exception ex) {};
			} while(cubeHandlerThreadControlElapsedTimeMS < MIN_CUBE_HANDLER_THREAD_LOOP_TIME_MS);
		}
	}
	
	public boolean isElevatorFaulted() {
		return false;
	}
	
	public double getElevatorPos() {
		return requestedElevatorPos;
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


package org.usfirst.frc.team195.robot.Subsystems;

import java.lang.Thread;

import java.util.List;

import org.usfirst.frc.team195.robot.Actions.IntakePositionAction;
import org.usfirst.frc.team195.robot.Actions.ShiftAction;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.Constants;
import org.usfirst.frc.team195.robot.Utilities.Controllers;
import org.usfirst.frc.team195.robot.Utilities.CustomSubsystem;
import org.usfirst.frc.team195.robot.Utilities.DriveHelper;
import org.usfirst.frc.team195.robot.Utilities.KnightJoystick;
import org.usfirst.frc.team195.robot.Utilities.Reportable;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class HIDControllerSubsystem implements CustomSubsystem {
	
	private static HIDControllerSubsystem instance = null;
	private static final int MIN_HID_THREAD_LOOP_TIME_MS = 20;

	public static HIDControllerSubsystem getInstance() {
		if(instance == null) {
			try {
				instance = new HIDControllerSubsystem();
			} catch (Exception ex) {
				ConsoleReporter.report(ex.toString(), MessageLevel.DEFCON1);
			}
		}
		
		return instance;
	}
	
	public static HIDControllerSubsystem getInstance(List<CustomSubsystem> subsystemList) {
		subsystemList.add(getInstance());
		return instance;
	}
	
	
	
	@Override
	public void init() {
		;
	}

	@Override
	public void subsystemHome() {
		;
	}

	@Override
	public void start() {
		runThread = true;
		driveJoyStickThread.start();
	}

	public void terminate() {
		runThread = false;
	}
	
//	@Override
//	public String toString() {
//		return generateReport();
//	}
	
	private HIDControllerSubsystem() throws Exception {
		super();
		ds = DriverStation.getInstance();

		Controllers robotControllers = Controllers.getInstance();
		driveJoystick = robotControllers.getDriveJoystick();

		driveBaseSubsystem = DriveBaseSubsystem.getInstance();
		cubeHandlerSubsystem = CubeHandlerSubsystem.getInstance();

		runThread = false;
		comingFromAuto = true;

		driveJoystickThreadControlStart = 0;
		driveJoystickThreadControlEnd = 0;
		driveJoystickThreadControlElapsedTimeMS = 0;
		
		driveJoyStickThread = new DriveJoyStickThread();
	}
	
	private DriveJoyStickThread driveJoyStickThread;
	
	protected DriveBaseSubsystem driveBaseSubsystem;
	protected CubeHandlerSubsystem cubeHandlerSubsystem;
	protected DriverStation ds;
	protected KnightJoystick driveJoystick;
	
	protected boolean runThread;

	protected boolean comingFromAuto;

	protected double driveJoystickThreadControlStart, driveJoystickThreadControlEnd;
	protected int driveJoystickThreadControlElapsedTimeMS;
	
	private class DriveJoyStickThread extends Thread {
		private double driveJoystickThreadControlStart, driveJoystickThreadControlEnd;
		private int driveJoystickThreadControlElapsedTimeMS;
		
		private DriveHelper driveHelper;
		private ShiftAction shiftAction;
		private IntakePositionAction intakePositionAction;

		private double x, y;
		
		public DriveJoyStickThread() {
			driveHelper = new DriveHelper();
			shiftAction = new ShiftAction();
			intakePositionAction = new IntakePositionAction();
			
			driveJoystickThreadControlStart = 0;
			driveJoystickThreadControlEnd = 0;
			driveJoystickThreadControlElapsedTimeMS = 0;
		}
		
		@Override
		public String toString() {
			String retVal = "";
			//retVal += "DriverXAxis:" + driveJoystick.getRawAxis(Constants.DRIVE_X_AXIS) + ";";
			//retVal += "DriverYAxis:" + driveJoystick.getRawAxis(Constants.DRIVE_Y_AXIS) + ";";
			return retVal;
		}
		
		@Override
		public void run() {
			while (!ds.isEnabled()) {try {Thread.sleep(20);} catch (Exception ex) {}}
			subsystemHome();

			while (ds.isAutonomous()) {try {Thread.sleep(100);} catch (Exception ex) {}}

			while(runThread) {
				driveJoystickThreadControlStart = Timer.getFPGATimestamp();

				if (driveJoystick.GetRisingEdgeButton(Constants.DRIVE_SHIFT_LOW)) {
					shiftAction.start(false);
				} else if (driveJoystick.GetRisingEdgeButton(Constants.DRIVE_SHIFT_HIGH)) {
					shiftAction.start(true);
				}
				
				/*if (driveJoystick.getRawButton(Constants.INTAKE_CLOSE_RUN)) {
					shiftAction.start(false);
					cubeHandlerSubsystem.setIntakeControl(IntakeControl.FORWARD);
				} else if (driveJoystick.GetRisingEdgeButton(Constants.INTAKE_OPEN)) {
					shiftAction.start(true);
				} else if (driveJoystick.getRawButton(Constants.INTAKE_RUN_REVERSE)) {
					cubeHandlerSubsystem.setIntakeControl(IntakeControl.REVERSE);
				} else if (driveJoystick.getRawButton(Constants.INTAKE_CLOSE)) {
					shiftAction.start(false);
				} else if(driveJoystick.getRawButton(Constants.INTAKE_RUN)) {
					cubeHandlerSubsystem.setIntakeControl(IntakeControl.FORWARD);
				} else {
					cubeHandlerSubsystem.setIntakeControl(IntakeControl.OFF);
				}*/
				
				if (driveJoystick.getRawButton(Constants.INTAKE_CLOSE)) {
					shiftAction.start(true);
					intakePositionAction.start(false);
				} else if (driveJoystick.getRawButton(Constants.INTAKE_OPEN)) {
					intakePositionAction.start(true);
					shiftAction.start(false);
				} else if (driveJoystick.getRawButton(Constants.INTAKE_CLOSE_HALF)) {
					intakePositionAction.start(true);
					shiftAction.start(true);
				} else if (driveJoystick.getRawButton(Constants.INTAKE_RUN_REVERSE)) {
					cubeHandlerSubsystem.setIntakeControl(IntakeControl.REVERSE);
				} else if(driveJoystick.getRawButton(Constants.INTAKE_RUN)) {
					cubeHandlerSubsystem.setIntakeControl(IntakeControl.FORWARD);
				} else {
					cubeHandlerSubsystem.setIntakeControl(IntakeControl.OFF);
				}

				x = driveJoystick.getRawAxis(Constants.DRIVE_X_AXIS);
				y = -driveJoystick.getRawAxis(Constants.DRIVE_Y_AXIS);

				driveBaseSubsystem.setBrakeMode(driveJoystick.getRawButton(Constants.DRIVE_HOLD_BRAKE));

				driveBaseSubsystem.setDriveSpeed(driveHelper.calculateOutput(y, x, driveJoystick.getRawButton(Constants.DRIVE_IMM_TURN), driveBaseSubsystem.isHighGear()));
				//driveBaseSubsystem.setDriveSpeed((y + x) * 2300, (y - x) * 2300);

				do {
					driveJoystickThreadControlEnd = Timer.getFPGATimestamp();
					driveJoystickThreadControlElapsedTimeMS = (int) ((driveJoystickThreadControlEnd - driveJoystickThreadControlStart) * 1000);
					if (driveJoystickThreadControlElapsedTimeMS < MIN_HID_THREAD_LOOP_TIME_MS)
						try{Thread.sleep(MIN_HID_THREAD_LOOP_TIME_MS - driveJoystickThreadControlElapsedTimeMS);}catch(Exception ex) {};
				} while(driveJoystickThreadControlElapsedTimeMS < MIN_HID_THREAD_LOOP_TIME_MS);

			}
		}
	}

//	@Override
//	public String generateReport() {
//		String retVal = "";
//		retVal += driveJoyStickThread.toString();
//		return retVal;
//	}
}



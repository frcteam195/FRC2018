package org.usfirst.frc.team195.robot;

import org.usfirst.frc.team195.robot.Actions.IntakePositionAction;
import org.usfirst.frc.team195.robot.Actions.ShiftAction;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Subsystems.CubeHandlerSubsystem;
import org.usfirst.frc.team195.robot.Subsystems.DriveBaseSubsystem;
import org.usfirst.frc.team195.robot.Utilities.*;

import edu.wpi.first.wpilibj.DriverStation;

public class HIDController implements Runnable {
	
	private static HIDController instance = null;
	private static final int MIN_HID_THREAD_LOOP_TIME_MS = 20;

	public static HIDController getInstance() {
		if(instance == null) {
			try {
				instance = new HIDController();
			} catch (Exception ex) {
				ConsoleReporter.report(ex, MessageLevel.DEFCON1);
			}
		}
		
		return instance;
	}

	
//	@Override
//	public String toString() {
//		return generateReport();
//	}
	
	private HIDController() throws Exception {
		super();
		ds = DriverStation.getInstance();

		Controllers robotControllers = Controllers.getInstance();
		driveJoystick = robotControllers.getDriveJoystick();

		driveBaseSubsystem = DriveBaseSubsystem.getInstance();
		cubeHandlerSubsystem = CubeHandlerSubsystem.getInstance();

		runThread = false;
		comingFromAuto = true;

		driveHelper = new DriveHelper();
		shiftAction = new ShiftAction();
		intakePositionAction = new IntakePositionAction();
	}
	

	protected DriveBaseSubsystem driveBaseSubsystem;
	protected CubeHandlerSubsystem cubeHandlerSubsystem;
	protected DriverStation ds;
	protected KnightJoystick driveJoystick;
	
	protected boolean runThread;

	protected boolean comingFromAuto;

	@Override
	public void run() {

		if (driveJoystick.GetRisingEdgeButton(Constants.DRIVE_SHIFT_LOW)) {
			shiftAction.start(false);
		} else if (driveJoystick.GetRisingEdgeButton(Constants.DRIVE_SHIFT_HIGH)) {
			shiftAction.start(true);
		}

			/*if (driveJoystick.getRawButton(Constants.INTAKE_CLOSE_RUN)) {
				shiftAction.start(false);
				cubeHandlerSubsystem.setmIntakeControl(IntakeControl.FORWARD);
			} else if (driveJoystick.GetRisingEdgeButton(Constants.INTAKE_OPEN)) {
				shiftAction.start(true);
			} else if (driveJoystick.getRawButton(Constants.INTAKE_RUN_REVERSE)) {
				cubeHandlerSubsystem.setmIntakeControl(IntakeControl.REVERSE);
			} else if (driveJoystick.getRawButton(Constants.INTAKE_CLOSE)) {
				shiftAction.start(false);
			} else if(driveJoystick.getRawButton(Constants.INTAKE_RUN)) {
				cubeHandlerSubsystem.setmIntakeControl(IntakeControl.FORWARD);
			} else {
				cubeHandlerSubsystem.setmIntakeControl(IntakeControl.OFF);
			}*/

//				if (driveJoystick.getRawButton(Constants.INTAKE_CLOSE)) {
//					shiftAction.start(true);
//					intakePositionAction.start(false);
//				} else if (driveJoystick.getRawButton(Constants.INTAKE_OPEN)) {
//					intakePositionAction.start(true);
//					shiftAction.start(false);
//				} else if (driveJoystick.getRawButton(Constants.INTAKE_CLOSE_HALF)) {
//					intakePositionAction.start(true);
//					shiftAction.start(true);
//				} else if (driveJoystick.getRawButton(Constants.INTAKE_RUN_REVERSE)) {
//					cubeHandlerSubsystem.setmIntakeControl(IntakeControl.REVERSE);
//				} else if(driveJoystick.getRawButton(Constants.INTAKE_RUN)) {
//					cubeHandlerSubsystem.setmIntakeControl(IntakeControl.FORWARD);
//				} else {
//					cubeHandlerSubsystem.setmIntakeControl(IntakeControl.OFF);
//				}

		x = driveJoystick.getRawAxis(Constants.DRIVE_X_AXIS);
		y = -driveJoystick.getRawAxis(Constants.DRIVE_Y_AXIS);

		driveBaseSubsystem.setBrakeMode(driveJoystick.getRawButton(Constants.DRIVE_HOLD_BRAKE));

		driveBaseSubsystem.setDriveOpenLoop(driveHelper.calculateOutput(y, x, driveJoystick.getRawButton(Constants.DRIVE_IMM_TURN), driveBaseSubsystem.isHighGear()));
		//driveBaseSubsystem.setDriveVelocity(driveHelper.calculateOutput(y, x, driveJoystick.getRawButton(Constants.DRIVE_IMM_TURN), driveBaseSubsystem.isHighGear(), 10000));
		//driveBaseSubsystem.setDriveVelocity(new DriveMotorValues((y + x) * 380, (y - x) * 380));

	}

	private ThreadRateControl threadRateControl = new ThreadRateControl();

	private DriveHelper driveHelper;
	private ShiftAction shiftAction;
	private IntakePositionAction intakePositionAction;

	private double x, y;

	@Override
	public String toString() {
		String retVal = "";
		//retVal += "DriverXAxis:" + driveJoystick.getRawAxis(Constants.DRIVE_X_AXIS) + ";";
		//retVal += "DriverYAxis:" + driveJoystick.getRawAxis(Constants.DRIVE_Y_AXIS) + ";";
		return retVal;
	}


//	@Override
//	public String generateReport() {
//		String retVal = "";
//		retVal += driveJoyStickThread.toString();
//		return retVal;
//	}
}



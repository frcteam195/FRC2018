package org.usfirst.frc.team195.robot;

import org.usfirst.frc.team195.robot.Actions.IntakePositionAction;
import org.usfirst.frc.team195.robot.Actions.ShiftAction;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Subsystems.CubeHandlerSubsystem;
import org.usfirst.frc.team195.robot.Subsystems.DriveBaseSubsystem;
import org.usfirst.frc.team195.robot.Utilities.*;

import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team195.robot.Utilities.Drivers.KnightJoystick;
import org.usfirst.frc.team195.robot.Utilities.Drivers.PolarArmControlJoystick;

public class HIDController implements Runnable {
	
	private static final int MIN_HID_THREAD_LOOP_TIME_MS = 20;
	private static HIDController instance = null;

	private DriveBaseSubsystem driveBaseSubsystem;
	private CubeHandlerSubsystem cubeHandlerSubsystem;
	private DriverStation ds;
	private KnightJoystick driveJoystick;
	private PolarArmControlJoystick armControlJoystick;
	private DriveHelper driveHelper;
	private ShiftAction shiftAction;
	private IntakePositionAction intakePositionAction;

	private HIDController() throws Exception {
		super();
		ds = DriverStation.getInstance();

		Controllers robotControllers = Controllers.getInstance();
		driveJoystick = robotControllers.getDriveJoystick();
		armControlJoystick = robotControllers.getArmControlJoystick();

		driveBaseSubsystem = DriveBaseSubsystem.getInstance();
		cubeHandlerSubsystem = CubeHandlerSubsystem.getInstance();

		driveHelper = new DriveHelper();
		shiftAction = new ShiftAction();
		intakePositionAction = new IntakePositionAction();
	}

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

	@Override
	public void run() {

		if (driveJoystick.getRisingEdgeButton(Constants.DRIVE_SHIFT_LOW)) {
			shiftAction.start(false);
		} else if (driveJoystick.getRisingEdgeButton(Constants.DRIVE_SHIFT_HIGH)) {
			shiftAction.start(true);
		}

			/*if (driveJoystick.getRawButton(Constants.INTAKE_CLOSE_RUN)) {
				shiftAction.start(false);
				cubeHandlerSubsystem.setmIntakeControl(IntakeControl.FORWARD);
			} else if (driveJoystick.getRisingEdgeButton(Constants.INTAKE_OPEN)) {
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

		double x = driveJoystick.getRawAxis(Constants.DRIVE_X_AXIS);
		double y = -driveJoystick.getRawAxis(Constants.DRIVE_Y_AXIS);

		driveBaseSubsystem.setBrakeMode(driveJoystick.getRawButton(Constants.DRIVE_HOLD_BRAKE));

		driveBaseSubsystem.setDriveOpenLoop(driveHelper.calculateOutput(y, x, driveJoystick.getRawButton(Constants.DRIVE_IMM_TURN), driveBaseSubsystem.isHighGear()));
		//driveBaseSubsystem.setDriveVelocity(driveHelper.calculateOutput(y, x, driveJoystick.getRawButton(Constants.DRIVE_IMM_TURN), driveBaseSubsystem.isHighGear(), 10000));
		//driveBaseSubsystem.setDriveVelocity(new DriveMotorValues((y + x) * 380, (y - x) * 380));

	}

}



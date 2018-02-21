package org.usfirst.frc.team195.robot;

import org.usfirst.frc.team195.robot.Actions.IntakePositionAction;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Subsystems.CubeHandlerSubsystem;
import org.usfirst.frc.team195.robot.Subsystems.DriveBaseSubsystem;
import org.usfirst.frc.team195.robot.Utilities.*;

import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.Arm.ArmConfiguration;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.ElevatorPosition;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.IntakeControl;
import org.usfirst.frc.team195.robot.Utilities.Drivers.KnightJoystick;
import org.usfirst.frc.team195.robot.Utilities.Drivers.PolarArmControlJoystick;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Util;

public class HIDController implements Runnable {
	private static HIDController instance = null;

	private DriveBaseSubsystem driveBaseSubsystem;
	private CubeHandlerSubsystem cubeHandlerSubsystem;
	private DriverStation ds;
	private KnightJoystick driveJoystickThrottle;
	//private KnightJoystick driveJoystickWheel;
	private PolarArmControlJoystick armControlJoystick;
	private KnightJoystick buttonBox1;
	private DriveHelper driveHelper;
	//private ShiftAction shiftAction;
	private IntakePositionAction intakePositionAction;

	private HIDController() throws Exception {
		ds = DriverStation.getInstance();

		Controllers robotControllers = Controllers.getInstance();
		driveJoystickThrottle = robotControllers.getDriveJoystickThrottle();
		//driveJoystickWheel = robotControllers.getDriveJoystickWheel();
		armControlJoystick = robotControllers.getArmControlJoystick();
		buttonBox1 = robotControllers.getButtonBox1();

		driveBaseSubsystem = DriveBaseSubsystem.getInstance();
		cubeHandlerSubsystem = CubeHandlerSubsystem.getInstance();

		driveHelper = new DriveHelper();
		//shiftAction = new ShiftAction();
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
		//NO SHIFTER THIS YEAR!!!
//		if (driveJoystickThrottle.getRisingEdgeButton(Constants.DRIVE_SHIFT_LOW)) {
//			shiftAction.start(false);
//		} else if (driveJoystickThrottle.getRisingEdgeButton(Constants.DRIVE_SHIFT_HIGH)) {
//			shiftAction.start(true);
//		}

		if (armControlJoystick.getRisingEdgeButton(Constants.ARM_MANUAL_POSITION_CONTROL))
			armControlJoystick.start(cubeHandlerSubsystem.getArmCoordinate());
		if (armControlJoystick.getRawButton(Constants.ARM_MANUAL_POSITION_CONTROL))
			cubeHandlerSubsystem.setArmCoordinate(armControlJoystick.getPolarMappingFromJoystick());

		if (armControlJoystick.getRawButton(Constants.ARM_INTAKE_IN))
			cubeHandlerSubsystem.setIntakeControl(IntakeControl.INTAKE_IN);
		else if (buttonBox1.getRawButton(Constants.BB1_INTAKE_OUT))
			cubeHandlerSubsystem.setIntakeControl(IntakeControl.INTAKE_OUT);
		else
			cubeHandlerSubsystem.setIntakeControl(IntakeControl.OFF);

		if (buttonBox1.getRisingEdgeButton(Constants.BB1_INTAKE_CLAMP))
			cubeHandlerSubsystem.setIntakeClamp(false);
		else if (buttonBox1.getRisingEdgeButton(Constants.BB1_INTAKE_UNCLAMP))
			cubeHandlerSubsystem.setIntakeClamp(true);

		if (buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_HOME))
			cubeHandlerSubsystem.setElevatorHeight(ElevatorPosition.HOME);
		else if (buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_SWITCH))
			cubeHandlerSubsystem.setElevatorHeight(ElevatorPosition.LOW);
		else if (buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_SCALE))
			cubeHandlerSubsystem.setElevatorHeight(ElevatorPosition.MID);
		else if (buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_SCALE_HIGH))
			cubeHandlerSubsystem.setElevatorHeight(ElevatorPosition.HIGH);
		else if (buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_INCREMENT))
			cubeHandlerSubsystem.incrementElevatorHeight();

		if (armControlJoystick.getRisingEdgeButton(Constants.ARM_STRAIGHT_OUT))
			cubeHandlerSubsystem.setArmCoordinate(ArmConfiguration.STRAIGHT);
		else if (armControlJoystick.getRisingEdgeButton(Constants.ARM_RIGHT_OUT))
			cubeHandlerSubsystem.setArmCoordinate(ArmConfiguration.RIGHT);
		else if (armControlJoystick.getRisingEdgeButton(Constants.ARM_LEFT_OUT))
			cubeHandlerSubsystem.setArmCoordinate(ArmConfiguration.LEFT);
		else if (armControlJoystick.getRisingEdgeButton(Constants.ARM_HOME))
			cubeHandlerSubsystem.setArmCoordinate(ArmConfiguration.HOME);

//		double wheel = driveJoystickThrottle.getRawAxis(Constants.DRIVE_X_AXIS);
//		double throttle = -driveJoystickThrottle.getRawAxis(Constants.DRIVE_Y_AXIS);

		double elevatorScaling = 1 - cubeHandlerSubsystem.getElevatorHeight() / Constants.kElevatorSoftMax;

		if (cubeHandlerSubsystem.getElevatorHeight() < 5)
			elevatorScaling = 1;
		else if (cubeHandlerSubsystem.getElevatorHeight() >= 16)
			elevatorScaling = 0.2;

		double x = QuickMaths.normalizeJoystickWithDeadband(driveJoystickThrottle.getRawAxis(Constants.DRIVE_X_AXIS), Constants.kJoystickDeadband) * elevatorScaling;
		double y = QuickMaths.normalizeJoystickWithDeadband(-driveJoystickThrottle.getRawAxis(Constants.DRIVE_Y_AXIS), Constants.kJoystickDeadband) * elevatorScaling;

		driveBaseSubsystem.setBrakeMode(driveJoystickThrottle.getRawButton(Constants.DRIVE_HOLD_BRAKE));

		driveBaseSubsystem.setDriveOpenLoop(new DriveMotorValues(Util.limit(y + x, 1), Util.limit(y - x, 1)));

//		driveBaseSubsystem.setDriveOpenLoop(driveHelper.calculateOutput(throttle, wheel, driveJoystickThrottle.getRawButton(Constants.DRIVE_IMM_TURN), driveBaseSubsystem.isHighGear()));
		//driveBaseSubsystem.setDriveVelocity(driveHelper.calculateOutput(y, x, driveJoystickThrottle.getRawButton(Constants.DRIVE_IMM_TURN), driveBaseSubsystem.isHighGear(), 10000));
		//driveBaseSubsystem.setDriveVelocity(new DriveMotorValues((y + x) * 380, (y - x) * 380));
	}

}



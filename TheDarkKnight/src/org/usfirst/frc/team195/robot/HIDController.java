package org.usfirst.frc.team195.robot;

import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team195.robot.Actions.AutomatedActions;
import org.usfirst.frc.team195.robot.Actions.IntakePositionAction;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Subsystems.ClimberSubsystem;
import org.usfirst.frc.team195.robot.Subsystems.CubeHandlerSubsystem;
import org.usfirst.frc.team195.robot.Subsystems.DriveBaseSubsystem;
import org.usfirst.frc.team195.robot.Utilities.*;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.*;
import org.usfirst.frc.team195.robot.Utilities.Drivers.KnightJoystick;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Util;

public class HIDController implements Runnable {
	private static HIDController instance = null;

	private DriveBaseSubsystem driveBaseSubsystem;
	private CubeHandlerSubsystem cubeHandlerSubsystem;
	private ClimberSubsystem climberSubsystem;
	private LEDController ledController;
	private DriverStation ds;
	private KnightJoystick driveJoystickThrottle;
	//private KnightJoystick driveJoystickWheel;
	private KnightJoystick armControlJoystick;
	private KnightJoystick buttonBox1;
	private KnightJoystick buttonBox2;
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
		buttonBox2 = robotControllers.getButtonBox2();

		driveBaseSubsystem = DriveBaseSubsystem.getInstance();
		cubeHandlerSubsystem = CubeHandlerSubsystem.getInstance();
		climberSubsystem = ClimberSubsystem.getInstance();
		ledController = LEDController.getInstance();

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


		if (armControlJoystick.getRawButton(Constants.ARM_INTAKE_IN))
			cubeHandlerSubsystem.setIntakeControl(IntakeControl.INTAKE_IN);
		else if (armControlJoystick.getRawButton(Constants.ARM_INTAKE_OUT))
			cubeHandlerSubsystem.setIntakeControl(IntakeControl.INTAKE_OUT);
		else if (armControlJoystick.getRawButton(Constants.ARM_INTAKE_OUT_HALFSPEED))
			cubeHandlerSubsystem.setIntakeControl(IntakeControl.INTAKE_OUT_HALFSPEED);
		else
			cubeHandlerSubsystem.setIntakeControl(IntakeControl.OFF);

		if (armControlJoystick.getRisingEdgeButton(Constants.ARM_INTAKE_CLAMP))
			cubeHandlerSubsystem.setIntakeClamp(false);
		else if (armControlJoystick.getRisingEdgeButton(Constants.ARM_INTAKE_UNCLAMP))
			cubeHandlerSubsystem.setIntakeClamp(true);

//		if (buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_HOME))
//			cubeHandlerSubsystem.setElevatorHeight(ElevatorPosition.HOME);
//		else if (buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_SWITCH))
//			cubeHandlerSubsystem.setElevatorHeight(ElevatorPosition.LOW);
//		else if (buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_SCALE))
//			cubeHandlerSubsystem.setElevatorHeight(ElevatorPosition.MID);
//		else if (buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_SCALE_HIGH))
//			cubeHandlerSubsystem.setElevatorHeight(ElevatorPosition.HIGH);
//		else if (buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_INCREMENT))
//			cubeHandlerSubsystem.incrementElevatorHeight();
//		else if (buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_DECREMENT))
//			cubeHandlerSubsystem.decrementElevatorHeight();
//
//		if (buttonBox1.getRisingEdgeButton(Constants.BB1_ARM_DOWN))
//			cubeHandlerSubsystem.setArmRotationDeg(ArmPosition.DOWN);
//		else if (armControlJoystick.getRisingEdgeButton(Constants.ARM_ARM_VERTICAL))
//			cubeHandlerSubsystem.setArmRotationDeg(ArmPosition.VERTICAL);
//		else if (buttonBox1.getRisingEdgeButton(Constants.BB1_ARM_BACK))
//			cubeHandlerSubsystem.setArmRotationDeg(ArmPosition.BACK);
//		else if (buttonBox1.getRisingEdgeButton(Constants.BB1_ARM_SWITCH))
//			cubeHandlerSubsystem.setArmRotationDeg(ArmPosition.SWITCH);
//
//		if (buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_REHOME)) {
//			ConsoleReporter.report("Elevator rehoming requested!", MessageLevel.DEFCON1);
//			cubeHandlerSubsystem.setElevatorControl(ElevatorControl.HOMING);
//		}

		if(buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_HOME))
			cubeHandlerSubsystem.setElevatorHeight(ElevatorPosition.HOME);
		else if(buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_SCALE))
			cubeHandlerSubsystem.setElevatorHeight(ElevatorPosition.MID);
		else if(buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_OVER_BACK_LOW))
			new TeleopActionRunner(AutomatedActions.PreparePlaceCubeOnScaleOverBackLow(), Constants.kActionTimeoutS).runAction();
		else if(buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_OVER_BACK_MID))
			new TeleopActionRunner(AutomatedActions.PreparePlaceCubeOnScaleOverBackMid(), Constants.kActionTimeoutS).runAction();
		else if(buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_OVER_BACK_HIGH))
			new TeleopActionRunner(AutomatedActions.PreparePlaceCubeOnScaleOverBackHigh(), Constants.kActionTimeoutS).runAction();
		else if(buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_SCALE_HIGH))
			cubeHandlerSubsystem.setElevatorHeight(ElevatorPosition.HIGH);

		if(armControlJoystick.getPOV() == Constants.ARM_ELEVATOR_INCREMENT_POV)
			cubeHandlerSubsystem.incrementElevatorHeight();
		else if(armControlJoystick.getPOV() == Constants.ARM_ELEVATOR_DECREMENT_POV)
			cubeHandlerSubsystem.decrementElevatorHeight();
		else if(armControlJoystick.getPOV() == Constants.ARM_ARM_LOW_POV)
			cubeHandlerSubsystem.setArmRotationDeg(ArmPosition.LOW);
		else if(armControlJoystick.getPOV() == Constants.ARM_ARM_MID_POV)
			cubeHandlerSubsystem.setArmRotationDeg(ArmPosition.MID);

		if (buttonBox1.getRisingEdgeButton(Constants.BB1_ARM_DOWN))
			cubeHandlerSubsystem.setArmRotationDeg(ArmPosition.DOWN);
		else if (armControlJoystick.getRisingEdgeButton(Constants.ARM_ARM_VERTICAL))
			cubeHandlerSubsystem.setArmRotationDeg(ArmPosition.VERTICAL);
		else if (buttonBox1.getRisingEdgeButton(Constants.BB1_ARM_BACK))
			cubeHandlerSubsystem.setArmRotationDeg(ArmPosition.BACK);
		else if (buttonBox1.getRisingEdgeButton(Constants.BB1_ARM_SWITCH))
			cubeHandlerSubsystem.setArmRotationDeg(ArmPosition.SWITCH);

		if (buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_REHOME)) {
			ConsoleReporter.report("Elevator rehoming requested!", MessageLevel.DEFCON1);
			cubeHandlerSubsystem.setElevatorControl(ElevatorControl.HOMING);
		}

		if (buttonBox1.getRisingEdgeButton(Constants.BB1_REQUEST_CUBE_FROM_WALL) || driveJoystickThrottle.getRisingEdgeButton(Constants.DRIVE_REQUEST_CUBE_FROM_WALL)) {
			if (ConnectionMonitor.getInstance().isConnected()) {
				ledController.configureBlink(4, LEDController.kDefaultBlinkDuration);
				ledController.setLEDColor(Constants.kRequestCubeColor);
				ledController.setRequestedState(LEDController.LEDState.BLINK);
			}
		}

		if (buttonBox2.getRisingEdgeButton(Constants.BB2_ARM_SET_ZERO) && cubeHandlerSubsystem.getArmControlMode() == ArmControl.OPEN_LOOP) {
			cubeHandlerSubsystem.setArmControl(ArmControl.HOMING);
			cubeHandlerSubsystem.setDisableCollisionPrevention(false);
		}

		if (buttonBox2.getRisingEdgeButton(Constants.BB2_ARM_SET_MANUAL)) {
			cubeHandlerSubsystem.setDisableCollisionPrevention(true);
			cubeHandlerSubsystem.setArmControl(ArmControl.OPEN_LOOP);
		}

//		if (buttonBox2.getRisingEdgeButton(Constants.BB2_ARM_REENABLE_POSITION)) {
//			cubeHandlerSubsystem.setDisableCollisionPrevention(false);
//			cubeHandlerSubsystem.setArmControl(ArmControl.POSITION);
//		}

//		if (buttonBox2.getRisingEdgeButton(Constants.BB2_ARM_ATTACH_HOOK)) {
//			ConsoleReporter.report("Attach hook pressed!");
//			(new TeleopActionRunner(AutomatedActions.AttachHook(), 4)).runAction();
//		}

		if (buttonBox2.getRisingEdgeButton(Constants.BB2_CLIMBER_DEPLOY_PLATFORM))
			climberSubsystem.deployPlatform();

		if (buttonBox2.getRawButton(Constants.BB2_CLIMBER_CLIMB_MAIN))
			climberSubsystem.climbMain(1);
		else
			climberSubsystem.climbMain(0);

		if (buttonBox2.getRawButton(Constants.BB2_CLIMBER_CLIMB_PITCH))
			climberSubsystem.climbPitch(1);
		else
			climberSubsystem.climbPitch(0);

		if (buttonBox2.getRawButton(8))
			climberSubsystem.climbMain(-0.5);
		else
			climberSubsystem.climbMain(0);

		if (buttonBox2.getRawButton(9))
			climberSubsystem.climbMain(-0.5);
		else
			climberSubsystem.climbPitch(0);

		cubeHandlerSubsystem.setArmOpenLoopDriveVal(QuickMaths.normalizeJoystickWithDeadband(armControlJoystick.getRawAxis(Constants.ARM_Y_AXIS), Constants.kJoystickDeadband)/4.0);

		double elevatorScaling = 1 - cubeHandlerSubsystem.getElevatorHeight() / Constants.kElevatorSoftMax;

		if (cubeHandlerSubsystem.getElevatorHeight() < 5)
			elevatorScaling = 1;

		elevatorScaling = elevatorScaling < 0.55 ? 0.55 : elevatorScaling;

		double x = QuickMaths.normalizeJoystickWithDeadband(driveJoystickThrottle.getRawAxis(Constants.DRIVE_X_AXIS), Constants.kJoystickDeadband) * elevatorScaling;
		double y = QuickMaths.normalizeJoystickWithDeadband(-driveJoystickThrottle.getRawAxis(Constants.DRIVE_Y_AXIS), Constants.kJoystickDeadband) * elevatorScaling;

		driveBaseSubsystem.setBrakeMode(driveJoystickThrottle.getRawButton(Constants.DRIVE_HOLD_BRAKE));

		driveBaseSubsystem.setDriveOpenLoop(new DriveMotorValues(Util.limit(y + x, 1), Util.limit(y - x, 1)));

		//		double wheel = driveJoystickThrottle.getRawAxis(Constants.DRIVE_X_AXIS);
//		double throttle = -driveJoystickThrottle.getRawAxis(Constants.DRIVE_Y_AXIS);

//		driveBaseSubsystem.setDriveOpenLoop(driveHelper.calculateOutput(throttle, wheel, driveJoystickThrottle.getRawButton(Constants.DRIVE_IMM_TURN), driveBaseSubsystem.isHighGear()));
		//driveBaseSubsystem.setDriveVelocity(driveHelper.calculateOutput(y, x, driveJoystickThrottle.getRawButton(Constants.DRIVE_IMM_TURN), driveBaseSubsystem.isHighGear(), 10000));
//		driveBaseSubsystem.setDriveVelocity(new DriveMotorValues((y + x) * 650, (y - x) * 650));
	}

}



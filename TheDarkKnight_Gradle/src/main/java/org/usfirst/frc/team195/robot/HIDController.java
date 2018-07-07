package org.usfirst.frc.team195.robot;

import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team195.robot.Actions.AutomatedActions;
import org.usfirst.frc.team195.robot.Actions.TurnToHeadingAction;
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


		///////////////////////////////
		//Arm Intake Control
		if (armControlJoystick.getRawButton(Constants.ARM_INTAKE_IN))
			cubeHandlerSubsystem.setIntakeControl(IntakeControl.INTAKE_IN);
		else if (armControlJoystick.getRawButton(Constants.ARM_INTAKE_OUT))
			cubeHandlerSubsystem.setIntakeControl(IntakeControl.INTAKE_OUT);
		else if (armControlJoystick.getRawButton(Constants.ARM_INTAKE_OUT_HALFSPEED))
			cubeHandlerSubsystem.setIntakeControl(IntakeControl.INTAKE_OUT_HALFSPEED);
		else
			cubeHandlerSubsystem.setIntakeControl(IntakeControl.OFF);
		///////////////////////////////

		///////////////////////////////
		//Arm Clamp Control
		if (armControlJoystick.getRisingEdgeButton(Constants.ARM_INTAKE_CLAMP))
			cubeHandlerSubsystem.setIntakeClamp(false);
		else if (armControlJoystick.getRisingEdgeButton(Constants.ARM_INTAKE_UNCLAMP))
			cubeHandlerSubsystem.setIntakeClamp(true);
		///////////////////////////////


		///////////////////////////////
		//Elevator Control
		if(buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_HOME)) {
			//TODO: Put back before IRI
			new TeleopActionRunner(AutomatedActions.SetRestingPosition(), Constants.kActionTimeoutS).runAction();
			//cubeHandlerSubsystem.setElevatorHeight(ElevatorPosition.GO_DOWN);
			//cubeHandlerSubsystem.setBlinkOnHome(false);
		}
		else if(buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_OVER_BACK_LOW))
			new TeleopActionRunner(AutomatedActions.PreparePlaceCubeOnScaleOverBackLow(), Constants.kActionTimeoutS).runAction();
		else if(buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_OVER_BACK_MID))
			new TeleopActionRunner(AutomatedActions.PreparePlaceCubeOnScaleOverBackMid(), Constants.kActionTimeoutS).runAction();
		else if(buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_OVER_BACK_HIGH))
			new TeleopActionRunner(AutomatedActions.PreparePlaceCubeOnScaleOverBackHigh(), Constants.kActionTimeoutS).runAction();
		else if(buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_SCALE_HIGH))
			cubeHandlerSubsystem.setElevatorHeight(ElevatorPosition.HIGH);
		else if(buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_LOW))
			cubeHandlerSubsystem.setElevatorHeight(ElevatorPosition.LOW);
		else if(buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_MID))
			cubeHandlerSubsystem.setElevatorHeight(ElevatorPosition.MID);
		else if(buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_HIGH))
			cubeHandlerSubsystem.setElevatorHeight(ElevatorPosition.HIGH);
		else if(buttonBox1.getRisingEdgeButton(Constants.BB1_AUTO_SWITCH))
			new TeleopActionRunner(AutomatedActions.PreparePlaceCubeOnSwitch(), Constants.kActionTimeoutS).runAction();
		else if (buttonBox2.getRisingEdgeButton(Constants.BB2_ELEVATOR_CALCULATED_SCALE))
			new TeleopActionRunner(AutomatedActions.PreparePlaceCubeCalculatedScaleHeight(ElevatorPosition.MID)).runAction();
		else if (buttonBox2.getRawButton(Constants.BB2_ELEVATOR_OPEN_LOOP_UP_RANDY)) {
			cubeHandlerSubsystem.setElevatorControl(ElevatorControl.MANUAL);
			cubeHandlerSubsystem.setOpenLoopElevator(0.5);
		}
		else if (buttonBox2.getRawButton(Constants.BB2_ELEVATOR_OPEN_LOOP_DOWN_RANDY)) {
			cubeHandlerSubsystem.setElevatorControl(ElevatorControl.MANUAL);
			cubeHandlerSubsystem.setOpenLoopElevator(-0.5);
		}
		else
			cubeHandlerSubsystem.setOpenLoopElevator(0);

		///////////////////////////////


		///////////////////////////////
		//Arm Angle Control POV
		if(armControlJoystick.getPOV() == Constants.ARM_ELEVATOR_INCREMENT_POV) {
			cubeHandlerSubsystem.setDisableFastDown(true);
			cubeHandlerSubsystem.incrementElevatorHeight();
		}
		else if(armControlJoystick.getPOV() == Constants.ARM_ELEVATOR_DECREMENT_POV) {
			cubeHandlerSubsystem.setDisableFastDown(true);
			cubeHandlerSubsystem.decrementElevatorHeight();
		}
		else if(armControlJoystick.getPOV() == Constants.ARM_ARM_LOW_POV) {
			cubeHandlerSubsystem.setArmRotationDeg(ArmPosition.LOW);
		}
		else if(armControlJoystick.getPOV() == Constants.ARM_ARM_MID_POV) {
			cubeHandlerSubsystem.setArmRotationDeg(ArmPosition.MID);
		} else {
			cubeHandlerSubsystem.setDisableFastDown(false);
		}
		///////////////////////////////

		///////////////////////////////
		//Arm Angle Control Buttons
		if (buttonBox1.getRisingEdgeButton(Constants.BB1_ARM_DOWN))
			cubeHandlerSubsystem.setArmRotationDeg(ArmPosition.DOWN);
		else if (armControlJoystick.getRisingEdgeButton(Constants.ARM_ARM_VERTICAL))
			cubeHandlerSubsystem.setArmRotationDeg(ArmPosition.VERTICAL);
		else if (buttonBox1.getRisingEdgeButton(Constants.BB1_ARM_BACK))
			cubeHandlerSubsystem.setArmRotationDeg(ArmPosition.BACK);
		else if (buttonBox1.getRisingEdgeButton(Constants.BB1_ARM_SWITCH))
			cubeHandlerSubsystem.setArmRotationDeg(ArmPosition.SWITCH);
		///////////////////////////////

		///////////////////////////////
		//Elevator Rehome
		if (buttonBox1.getRisingEdgeButton(Constants.BB1_ELEVATOR_REHOME)) {
			ConsoleReporter.report("Elevator rehoming requested!", MessageLevel.DEFCON1);
			cubeHandlerSubsystem.setBlinkOnHome(true);
			cubeHandlerSubsystem.setElevatorControl(ElevatorControl.HOMING);
		}
		///////////////////////////////

		///////////////////////////////
		//LED Request Cube
		if (buttonBox1.getRisingEdgeButton(Constants.BB1_REQUEST_CUBE_FROM_WALL) || driveJoystickThrottle.getRisingEdgeButton(Constants.DRIVE_REQUEST_CUBE_FROM_WALL)) {
			if (ConnectionMonitor.getInstance().isConnected()) {
				ledController.configureBlink(4, LEDController.kDefaultBlinkDuration);
				ledController.setLEDColor(Constants.kRequestCubeColor);
				ledController.setRequestedState(LEDController.LEDState.BLINK);
			}
		}
		///////////////////////////////

		///////////////////////////////
		//Arm Homing
		if (buttonBox2.getRisingEdgeButton(Constants.BB2_ARM_SET_ZERO) && cubeHandlerSubsystem.getArmControlMode() == ArmControl.OPEN_LOOP) {
			cubeHandlerSubsystem.setArmControl(ArmControl.HOMING);
			cubeHandlerSubsystem.setDisableCollisionPrevention(false);
		}

		if (buttonBox2.getRisingEdgeButton(Constants.BB2_ARM_SET_MANUAL)) {
			cubeHandlerSubsystem.setDisableCollisionPrevention(true);
			cubeHandlerSubsystem.setArmControl(ArmControl.OPEN_LOOP);
		}
		///////////////////////////////


		///////////////////////////////
		//Climber Deployment and Lifting
		if (buttonBox2.getRisingEdgeButton(Constants.BB2_CLIMBER_DEPLOY_PLATFORM))
			climberSubsystem.deployPlatform();


		if (buttonBox2.getRawButton(Constants.BB2_CLIMBER_CLIMB_ROLL_DEPLOY)) {
			cubeHandlerSubsystem.prepareCLimb();
			climberSubsystem.setVelocity(-0.5*Constants.kClimberMaxVelocity);
		}
		else if (buttonBox2.getRawButton(Constants.BB2_CLIMBER_CLIMB_IN)) {
			climberSubsystem.setVelocity(Constants.kClimberMaxVelocity);
			//ledController.setMessage("195", true, true);
		}
		else if (buttonBox2.getRawButton(Constants.BB2_CLIMBER_CLIMB_HOOK_SLOW)) {
			cubeHandlerSubsystem.prepareCLimb();
			climberSubsystem.setOpenLoop(0.55);
			//0.55
		}
		else {
			//Use velocity control set to 0 to hold robots up while enabled
			climberSubsystem.setVelocity(0);
		}
		///////////////////////////////

		//Manual arm control
		cubeHandlerSubsystem.setArmOpenLoopDriveVal(QuickMaths.normalizeJoystickWithDeadband(armControlJoystick.getRawAxis(Constants.ARM_Y_AXIS), Constants.kJoystickDeadband)/2.0);

		///////////////////////////////
		//Elevator mapped drive speed to help prevent tipping
		double elevatorScaling = 1 - cubeHandlerSubsystem.getElevatorHeight() / Constants.kElevatorSoftMax;

		if (cubeHandlerSubsystem.getElevatorHeight() < 5)
			elevatorScaling = 1;

		elevatorScaling = elevatorScaling < 0.55 ? 0.55 : elevatorScaling;

		double x = QuickMaths.normalizeJoystickWithDeadband(driveJoystickThrottle.getRawAxis(Constants.DRIVE_X_AXIS), Constants.kJoystickDeadband) * elevatorScaling;
		double y = QuickMaths.normalizeJoystickWithDeadband(-driveJoystickThrottle.getRawAxis(Constants.DRIVE_Y_AXIS), Constants.kJoystickDeadband) * elevatorScaling;

		driveBaseSubsystem.setBrakeMode(driveJoystickThrottle.getRawButton(Constants.DRIVE_HOLD_BRAKE));


		driveBaseSubsystem.setDriveOpenLoop(new DriveMotorValues(Util.limit(y + x, 1), Util.limit(y - x, 1)));
		///////////////////////////////

		//TODO: Enable for new drivers maybe
		//CheesyDrive for new drivers
//		driveBaseSubsystem.setDriveOpenLoop(driveHelper.calculateOutput(y, x, driveJoystickThrottle.getRawButton(Constants.DRIVE_IMM_TURN), driveBaseSubsystem.isHighGear()));



//		driveBaseSubsystem.setDriveVelocity(driveHelper.calculateOutput(y, x, driveJoystickThrottle.getRawButton(Constants.DRIVE_IMM_TURN), driveBaseSubsystem.isHighGear(), 10000));
//		driveBaseSubsystem.setDriveVelocity(new DriveMotorValues((y + x) * 650, (y - x) * 650));
	}

}



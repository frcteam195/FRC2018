package org.usfirst.frc.team195.robot;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeBase;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeExecuter;
import org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromCenter.Left.LeftFromCenterMode_3CubeSwitch;
import org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromCenter.Right.RightFromCenterMode_3CubeSwitch;
import org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromLeft.LeftLeft.LeftFromLeft3CubeScaleModeAlt;
import org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromLeft.RightRight.RightFromLeft3CubeScaleModeAlt;
import org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromRight.LeftFromRight.LeftFromRight3CubeScaleModeAlt;
import org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromRight.LeftRight.LeftRightFromRightMode_3cubeScale;
import org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromRight.LeftRight.LeftRightFromRightMode_3cubeScaleAlt;
import org.usfirst.frc.team195.robot.Autonomous.Modes.StartFromRight.RightRight.RightRightFromRight_5cubescale;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.DashboardReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Subsystems.ClimberSubsystem;
import org.usfirst.frc.team195.robot.Subsystems.CubeHandlerSubsystem;
import org.usfirst.frc.team195.robot.Subsystems.DriveBaseSubsystem;
import org.usfirst.frc.team195.robot.Utilities.*;
import org.usfirst.frc.team195.robot.Utilities.Loops.Looper;
import org.usfirst.frc.team195.robot.Utilities.Loops.RobotStateEstimator;

import java.util.ArrayList;

public class Robot extends RobbieRobot {
	private Controllers robotControllers;
	private ArrayList<CustomSubsystem> subsystemVector;

	private DriveBaseSubsystem driveBaseSubsystem;
	private RobotStateEstimator robotStateEstimator;
	private CubeHandlerSubsystem cubeHandlerSubsystem;
	private ClimberSubsystem climberSubsystem;
	private HIDController hidController;
	private LEDController ledController;
	private DashboardReporter dashboardReporter;
	private CriticalSystemsMonitor criticalSystemsMonitor;
	private AutoModeExecuter autoModeExecuter;
	private Looper mLooper;
	private GameSpecificMessageParser gameSpecificMessageParser;
	private ThreadRateControl threadRateControl = new ThreadRateControl();
	private AutoSelectionReceiver autoSelectionReceiver;
	
	public Robot() {
		;
	}

	@Override
	public void robotInit() {
		Thread.currentThread().setPriority(Constants.kRobotThreadPriority);

		//Setup the ConsoleReporter first so that subsystems can report errors if they occur
		ConsoleReporter.setReportingLevel(MessageLevel.INFO);
		ConsoleReporter.getInstance().start();
		ConsoleReporter.report("Console Reporter Running!", MessageLevel.INFO);

		ledController = LEDController.getInstance();
		ledController.start();
		ledController.setRequestedState(LEDController.LEDState.BLINK);

		ConnectionMonitor.getInstance().start();

		gameSpecificMessageParser = GameSpecificMessageParser.getInstance();

		autoSelectionReceiver = AutoSelectionReceiver.getInstance();
		autoSelectionReceiver.setPortNumber(Constants.AUTO_SELECTOR_PORT);
		autoSelectionReceiver.start();

		robotControllers = Controllers.getInstance();
		mLooper = new Looper();
		subsystemVector = new ArrayList<CustomSubsystem>();

		hidController = HIDController.getInstance();

		driveBaseSubsystem = DriveBaseSubsystem.getInstance(subsystemVector);
		cubeHandlerSubsystem = CubeHandlerSubsystem.getInstance(subsystemVector);
		climberSubsystem = ClimberSubsystem.getInstance(subsystemVector);

		threadRateControl.start(true);

		for (CustomSubsystem customSubsystem : subsystemVector) {
			customSubsystem.init();
			threadRateControl.doRateControl(100);
		}

		for (CustomSubsystem customSubsystem : subsystemVector) {
			customSubsystem.registerEnabledLoops(mLooper);
		}

		robotStateEstimator = RobotStateEstimator.getInstance();
		mLooper.register(robotStateEstimator);

		//Setup the DashboardReporter once all other subsystems have been initialized
		dashboardReporter = DashboardReporter.getInstance(subsystemVector);
		dashboardReporter.start();

		//Setup the CriticalSystemsMonitor once all other subsystems have been initialized
		criticalSystemsMonitor = CriticalSystemsMonitor.getInstance(subsystemVector);
		criticalSystemsMonitor.start();


		ConsoleReporter.report("Robot Init Complete!", MessageLevel.INFO);
	}

	@Override
	public void autonomous() {
		mLooper.start(true);
		driveBaseSubsystem.setBrakeMode(true);
		autoModeExecuter = new AutoModeExecuter();

		StartingPosition startingPosition = autoSelectionReceiver.getStartingPosition();
		// Don't terminate if you want to run auto again without rebooting code
//		autoSelectionReceiver.terminate();

		FieldLayout fieldLayout = gameSpecificMessageParser.getTargetFieldLayout();
		AutoModeBase autoMode = null;
		if (!gameSpecificMessageParser.isAutoDisabled() && fieldLayout != FieldLayout.UNDEFINED) {
			switch (startingPosition) {
				case LEFT:
					autoMode = getModeStartingLeft(fieldLayout);
					break;
				case RIGHT:
					autoMode = getModeStartingRight(fieldLayout);
					break;
				case CENTER:
					autoMode = getModeStartingCenter(fieldLayout);
					break;
				default:
					return;
			}
		}

		if (autoMode != null)
			autoModeExecuter.setAutoMode(autoMode);
		else
			return;

		autoModeExecuter.start();
		threadRateControl.start(true);

		while (isAutonomous() && isEnabled()) {threadRateControl.doRateControl(100);}
	}
	
	@Override
	protected void disabled() {
		exitAuto();

		mLooper.stop();

		threadRateControl.start(true);

		while (isDisabled()) {
			driveBaseSubsystem.setBrakeMode(false);
			threadRateControl.doRateControl(100);
		}
	}

	@Override
	public void operatorControl() {
		exitAuto();

		mLooper.start(false);
		driveBaseSubsystem.setDriveOpenLoop(DriveMotorValues.NEUTRAL);
		threadRateControl.start(true);

		while (isOperatorControl() && isEnabled()) {
			hidController.run();
			threadRateControl.doRateControl(20);
		}
	}

	@Override
	public void test() {
		ConsoleReporter.report("TESTING MODE ENTERED!", MessageLevel.DEFCON1);
		boolean systemPassedTest = true;

		for (CustomSubsystem customSubsystem : subsystemVector) {
			if (customSubsystem instanceof DiagnosableSubsystem && isEnabled() && isTest())
				systemPassedTest &= ((DiagnosableSubsystem) customSubsystem).runDiagnostics();
		}

		if (!systemPassedTest)
			ConsoleReporter.report("Robot has failed self diagnostics! Check the logs for more info.", MessageLevel.DEFCON1);
		else
			ConsoleReporter.report("SYSTEM PASSED TEST!", MessageLevel.DEFCON1);

		Timer.delay(4);

		//Crash the JVM and force the code to reset so we no longer have the motor controllers configured for testing
		System.exit(1);
	}

	private void exitAuto() {
		try {
			if (autoModeExecuter != null)
				autoModeExecuter.stop();

			autoModeExecuter = null;
		} catch (Throwable t) {
			ConsoleReporter.report(t, MessageLevel.ERROR);
		}
	}

	private AutoModeBase getModeStartingLeft(FieldLayout fieldLayout) {
		switch (fieldLayout) {
			case LEFT_LEFT:
				//return new LeftLeftFromLeftMode_3cube();
				return new LeftFromLeft3CubeScaleModeAlt();
			case LEFT_RIGHT:
				//return new RightFromLeft3CubeScaleMode();
				return new RightFromLeft3CubeScaleModeAlt();
			case RIGHT_LEFT:
				return new LeftFromLeft3CubeScaleModeAlt();
			case RIGHT_RIGHT:
				return new RightFromLeft3CubeScaleModeAlt();
			case UNDEFINED:
			default:
				break;
		}
		return null;
	}

	private AutoModeBase getModeStartingRight(FieldLayout fieldLayout) {
		switch (fieldLayout) {
			case LEFT_LEFT:
				//return new LeftLeftFromRightMode_2cube();
				//return new LeftLeftFromRight_2cubeScale();
				return new LeftFromRight3CubeScaleModeAlt();
			case LEFT_RIGHT:
				//return new LeftRightFromRightMode_3cubeScale();
				return new LeftRightFromRightMode_3cubeScale();
			case RIGHT_LEFT:
//				return new LeftLeftFromRight_2cubeScale();
				//return new RightLeftFromRightMode_2cube();
				//return new LeftFromRight3CubeScaleMode();
				return new LeftFromRight3CubeScaleModeAlt();
			case RIGHT_RIGHT:
				//return new LeftRightFromRightMode_3cubeScaleAlt();
				return new LeftRightFromRightMode_3cubeScaleAlt();
			case UNDEFINED:
			default:
				break;
		}
		return null;
	}

	private AutoModeBase getModeStartingCenter(FieldLayout fieldLayout) {
		switch (fieldLayout) {
			case LEFT_LEFT:
				//return new LeftFromCenterMode_2cube();
				//return new LeftLeftFromCenterMode_SwitchScale();
				//return new LeftFromCenterMode_2cube();
				return new LeftFromCenterMode_3CubeSwitch();
			case LEFT_RIGHT:
				//return new LeftFromCenterMode_2cube();
				//return new LeftFromCenterMode_2cube();
				return new LeftFromCenterMode_3CubeSwitch();
				//return new LeftRightFromCenterMode_SwitchScale();
			case RIGHT_LEFT:
				//return new RightFromCenterMode_2cube();
				//return new RightFromCenterMode_2cube();
				return new RightFromCenterMode_3CubeSwitch();
			case RIGHT_RIGHT:
				//return new RightFromCenterMode_2cube();
				//return new RightFromCenterMode_3cube();
				//return new RightFromCenterMode_2cube();
				return new RightFromCenterMode_3CubeSwitch();
			case UNDEFINED:
			default:
				break;
		}
		return null;
	}
}

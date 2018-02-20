package org.usfirst.frc.team195.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Autonomous.Framework.AutoModeExecuter;
import org.usfirst.frc.team195.robot.Autonomous.SwitchCubeThenScaleMode;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.DashboardReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Subsystems.*;
import org.usfirst.frc.team195.robot.Utilities.*;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.Arm.ArmConfiguration;
import org.usfirst.frc.team195.robot.Utilities.CubeHandler.Arm.PolarCoordinate;
import org.usfirst.frc.team195.robot.Utilities.Loops.Looper;
import org.usfirst.frc.team195.robot.Utilities.Loops.RobotStateEstimator;

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
	private ThreadRateControl threadRateControl = new ThreadRateControl();
	
	public Robot() {
		;
	}

	@Override
	public void robotInit() {
		//Setup the ConsoleReporter first so that subsystems can report errors if they occur
		ConsoleReporter.setReportingLevel(MessageLevel.INFO);
		ConsoleReporter.getInstance().start();
		ConsoleReporter.report("Console Reporter Running!", MessageLevel.INFO);

		ledController = LEDController.getInstance();
		ledController.start();
		ledController.setRequestedState(LEDController.LEDState.BLINK);

		ConnectionMonitor.getInstance().start();

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
		//dashboardReporter.start();

		//Setup the CriticalSystemsMonitor once all other subsystems have been initialized
		criticalSystemsMonitor = CriticalSystemsMonitor.getInstance(subsystemVector);
		criticalSystemsMonitor.start();


		ConsoleReporter.report("Robot Init Complete!", MessageLevel.INFO);
	}

	@Override
	public void autonomous() {
		mLooper.start();

		autoModeExecuter = new AutoModeExecuter();
		autoModeExecuter.setAutoMode(new SwitchCubeThenScaleMode());
		autoModeExecuter.start();

		threadRateControl.start(true);

		while (isAutonomous() && isEnabled()) {threadRateControl.doRateControl(100);}
	}
	
	@Override
	protected void disabled() {
		mLooper.stop();

		threadRateControl.start(true);

		while (isDisabled()) {
			threadRateControl.doRateControl(100);
		}
	}

	@Override
	public void operatorControl() {
		mLooper.start();
		driveBaseSubsystem.setControlMode(DriveControlState.OPEN_LOOP);
		threadRateControl.start(true);

		while (isOperatorControl() && isEnabled()) {
			hidController.run();

			//threadRateControl.doRateControl(2000);
//			cubeHandlerSubsystem.setArmCoordinate(new PolarCoordinate(16.75, 90));
//			threadRateControl.doRateControl(2000);
//			cubeHandlerSubsystem.setArmCoordinate(new PolarCoordinate(16.75, 135));
//			threadRateControl.doRateControl(2000);
//			cubeHandlerSubsystem.setArmCoordinate(new PolarCoordinate(16.75, 45));
//			threadRateControl.doRateControl(2000);
//			cubeHandlerSubsystem.setArmCoordinate(new PolarCoordinate(8, 45));
//			threadRateControl.doRateControl(2000);
//			cubeHandlerSubsystem.setArmCoordinate(new PolarCoordinate(8, 135));
//			threadRateControl.doRateControl(2000);
//			cubeHandlerSubsystem.setArmCoordinate(new PolarCoordinate(16.75, 135));
//			threadRateControl.doRateControl(2000);
//			cubeHandlerSubsystem.setArmCoordinate(new PolarCoordinate(8, 135));
//			threadRateControl.doRateControl(2000);
//			cubeHandlerSubsystem.setArmCoordinate(new PolarCoordinate(8, 45));
//			threadRateControl.doRateControl(2000);
//			cubeHandlerSubsystem.setArmCoordinate(new PolarCoordinate(16.75, 45));
//			threadRateControl.doRateControl(2000);
//			cubeHandlerSubsystem.setArmCoordinate(new PolarCoordinate(16.75, 90));
//			threadRateControl.doRateControl(2000);
//			cubeHandlerSubsystem.setArmCoordinate(ArmConfiguration.HOME);
//			threadRateControl.doRateControl(2000);

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
}

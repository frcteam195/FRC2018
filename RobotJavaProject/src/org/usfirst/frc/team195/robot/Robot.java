package org.usfirst.frc.team195.robot;

import java.io.Console;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import org.eclipse.jetty.io.Connection;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.DashboardReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Subsystems.*;
import org.usfirst.frc.team195.robot.Utilities.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathFollowerRobotState;

public class Robot extends RobbieRobot {
	private Controllers robotControllers;
	private ArrayList<CustomSubsystem> subsystemVector;

	private DriveBaseSubsystem driveBaseSubsystem;
	private RobotStateEstimator robotStateEstimator;
	private CubeHandlerSubsystem cubeHandlerSubsystem;
	private HIDControllerSubsystem hidControllerSubsystem;
	private LEDControllerSubsystem ledControllerSubsystem;
	private ConnectionMonitorSubsystem connectionMonitorSubsystem;
	private DashboardReporter dashboardReporter;
	
	public Robot() {
		;
	}

	@Override
	public void robotInit() {
		//Setup the ConsoleReporter first so that subsystems can report errors if they occur
		ConsoleReporter.setReportingLevel(MessageLevel.INFO);
		ConsoleReporter.getInstance().start();
		ConsoleReporter.report("Console Reporter Running!", MessageLevel.INFO);

		robotControllers = Controllers.getInstance();
		subsystemVector = new ArrayList<CustomSubsystem>();
		
		driveBaseSubsystem = DriveBaseSubsystem.getInstance(subsystemVector);
		robotStateEstimator = RobotStateEstimator.getInstance(subsystemVector);
		//cubeHandlerSubsystem = CubeHandlerSubsystem.getInstance(subsystemVector);
		hidControllerSubsystem = HIDControllerSubsystem.getInstance(subsystemVector);
		connectionMonitorSubsystem = ConnectionMonitorSubsystem.getInstance(subsystemVector);
		ledControllerSubsystem = LEDControllerSubsystem.getInstance(subsystemVector);
		
		for (CustomSubsystem customSubsystem : subsystemVector) {
			customSubsystem.init();
		}

		for (CustomSubsystem customSubsystem : subsystemVector) {
			customSubsystem.start();
		}
		
		//Setup the DashboardReporter once all other subsystems have been initialized
		dashboardReporter = DashboardReporter.getInstance(subsystemVector);
		dashboardReporter.start();

	}

	@Override
	public void autonomous() {

		while (isAutonomous() && isEnabled()) {try{Thread.sleep(100);}catch(Exception ex) {}}
	}
	
	@Override
	protected void disabled() { ; }

	@Override
	public void operatorControl() {
		driveBaseSubsystem.setControlMode(DriveControlState.VELOCITY);
		while (isOperatorControl() && isEnabled()) {try{Thread.sleep(100);}catch(Exception ex) {}}
	}

	@Override
	public void test() {
		ConsoleReporter.report("TESTING MODE ENTERED!", MessageLevel.DEFCON1);
		boolean systemPassedTest = true;

		for (CustomSubsystem customSubsystem : subsystemVector) {
			if (customSubsystem instanceof DiagnosableSubsystem)
				systemPassedTest &= ((DiagnosableSubsystem) customSubsystem).runDiagnostics();
		}

		if (!systemPassedTest)
			ConsoleReporter.report("Robot has failed self diagnostics! Check the logs for more info.", MessageLevel.DEFCON1);
		else
			ConsoleReporter.report("SYSTEM PASSED TEST!", MessageLevel.DEFCON1);

		Timer.delay(4);

		//TODO: Test RIO crash code
		//Crash the JVM and force the code to reset so we no longer have the motor controllers configured for testing
		System.exit(1);
	}
}

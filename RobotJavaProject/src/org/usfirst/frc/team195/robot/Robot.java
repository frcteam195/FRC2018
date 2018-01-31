package org.usfirst.frc.team195.robot;

import java.util.ArrayList;

import org.usfirst.frc.team195.robot.Autonomous.AutoProfileTest3;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.DashboardReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Subsystems.*;
import org.usfirst.frc.team195.robot.Utilities.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Robot extends RobbieRobot {
	private Controllers robotControllers;
	private ArrayList<CustomSubsystem> subsystemVector;

	private AutoProfileTest3 autoProfileTest;

	private DriveBaseSubsystem driveBaseSubsystem;
	private RobotStateEstimator robotStateEstimator;
	private CubeHandlerSubsystem cubeHandlerSubsystem;
	private HIDControllerSubsystem hidControllerSubsystem;
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
		
		for (CustomSubsystem customSubsystem : subsystemVector) {
			customSubsystem.init();
		}
		
		//Setup the DashboardReporter once all other subsystems have been initialized
		dashboardReporter = DashboardReporter.getInstance(subsystemVector);
		dashboardReporter.start();


		
		//autoProfileTest = new AutoProfileTest3();
	}

	@Override
	public void autonomous() {
		for (CustomSubsystem customSubsystem : subsystemVector) {
			customSubsystem.subsystemHome();
			customSubsystem.start();
		}

		//autoProfileTest.start();
		
		while (isAutonomous() && isEnabled()) {try{Thread.sleep(100);}catch(Exception ex) {}}
	}
	
	@Override
	protected void disabled() {
		for (CustomSubsystem customSubsystem : subsystemVector) {
			customSubsystem.terminate();
		};
	}

	@Override
	public void operatorControl() {
		for (CustomSubsystem customSubsystem : subsystemVector) {
			customSubsystem.start();
		}

		driveBaseSubsystem.setControlMode(DriveControlState.OPEN_LOOP);
		while (isOperatorControl() && isEnabled()) {try{Thread.sleep(100);}catch(Exception ex) {}}
	}

	@Override
	public void test() {
		boolean retVal = true;

		for (CustomSubsystem customSubsystem : subsystemVector) {
			if (customSubsystem instanceof DiagnosableSubsystem)
				retVal &= ((DiagnosableSubsystem) customSubsystem).runDiagnostics();
		}

		if (!retVal)
			ConsoleReporter.report("Robot has failed self diagnostics! Check the logs for more info.", MessageLevel.DEFCON1);
	}
}

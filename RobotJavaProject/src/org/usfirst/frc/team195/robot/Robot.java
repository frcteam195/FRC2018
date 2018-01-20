package org.usfirst.frc.team195.robot;

import java.util.ArrayList;

import org.usfirst.frc.team195.robot.Autonomous.AutoProfileTest;
import org.usfirst.frc.team195.robot.Subsystems.*;
import org.usfirst.frc.team195.robot.Utilities.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Robot extends RobbieRobot {
	private Controllers robotControllers;
	private ArrayList<CustomSubsystem> subsystemVector;

	private AutoProfileTest autoProfileTest;

	private DriveBaseSubsystem driveBaseSubsystem;
	private CubeHandlerSubsystem cubeHandlerSubsystem;
	private HIDControllerSubsystem hidControllerSubsystem;
	private DashboardReporter dashboardReporter;

	private CKAutoBuilder<TalonSRX> ckAuto;
	
	
	public Robot() {

	}

	@Override
	public void robotInit() {
		robotControllers = Controllers.getInstance();
		subsystemVector = new ArrayList<CustomSubsystem>();
		
		driveBaseSubsystem = DriveBaseSubsystem.getInstance(subsystemVector);
		cubeHandlerSubsystem = CubeHandlerSubsystem.getInstance(subsystemVector);
		hidControllerSubsystem = HIDControllerSubsystem.getInstance(subsystemVector);
		
		for (CustomSubsystem customSubsystem : subsystemVector) {
			customSubsystem.init();
		}
		
		for (CustomSubsystem customSubsystem : subsystemVector) {
			customSubsystem.start();
		}
		
		//Setup the DashboardReporter once all other subsystems have been instantiated
		dashboardReporter = DashboardReporter.getInstance(subsystemVector);
		dashboardReporter.start();

//		try {
//			ckAuto = new CKAutoBuilder<TalonSRX>(robotControllers.getLeftDrive1(), robotControllers.getRightDrive1(), this);
//		} catch (Exception ex) {
//
//		}
		
		autoProfileTest = new AutoProfileTest();
	}

	@Override
	public void autonomous() {
//		ckAuto.addAutoStep(0, 0, 2000);	//Delay for two seconds
//		ckAuto.addAutoStep(1, 0.25, 1000);	//Drive forward while turning right for one second
//		ckAuto.addAutoStep(0.8, -0.5, 700);	//Drive forward while turning left for 600 milliseconds
//		ckAuto.addAutoStep(0.75, 0, 500);	//Drive forward for half a second
//		ckAuto.addAutoStep(0, 0, 200);	//Stop
//		ckAuto.start();

		autoProfileTest.start();
		
		while (isAutonomous() && isEnabled()) {try{Thread.sleep(100);}catch(Exception ex) {}}
	}

	@Override
	public void operatorControl() {
		driveBaseSubsystem.setControlMode(ControlMode.PercentOutput);
		while (isOperatorControl() && isEnabled()) {try{Thread.sleep(100);}catch(Exception ex) {}}
	}

	@Override
	public void test() {
	}
}

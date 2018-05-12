package com.team195;

import com.team195.subsystems.*;
import com.team195.utils.RobbieRobot;

public class Robot extends RobbieRobot {

	private DriveControl oi;
	private PIDController pidController;
	private VisionReceiver visionReceiver;

	@Override
	public void robotInit() {
		oi = DriveControl.getInstance();
		pidController = PIDController.getInstance();
		visionReceiver = VisionReceiver.getInstance();
		//System.out.println("Init DriveControl");
		oi.start();
		pidController.start();
		visionReceiver.start();
	}

	@Override
	public void autonomous() {
	}

	@Override
	public void operatorControl() {
		while (isEnabled()) {
			;
		}
	}

	@Override
	public void test() {
	}
}

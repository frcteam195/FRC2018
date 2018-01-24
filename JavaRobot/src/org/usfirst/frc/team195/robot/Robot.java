
package org.usfirst.frc.team195.robot;

import edu.wpi.first.wpilibj.SampleRobot;


public class Robot extends SampleRobot {

	public Robot() {
	}

	@Override
	public void robotInit() {
		myDrive = new DriveBase();
		myIntake = new Intake();
		
	}
	
	@Override
	public void autonomous() {

	}

	@Override
	public void operatorControl() {
		while (isOperatorControl() && isEnabled()) {
			myDrive.run();
			myIntake.run();
		}
	}


	@Override
	public void test() {
	}
	
	private DriveBase myDrive;
	private Intake myIntake;
}

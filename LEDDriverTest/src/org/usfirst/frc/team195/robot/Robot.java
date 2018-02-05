package org.usfirst.frc.team195.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends SampleRobot {
	private LEDDriverRGB ledController;

	public Robot() {
	}

	@Override
	public void robotInit() {
		ledController = LEDDriverRGB.getInstance();
		ledController.start();
		ledController.setLEDColor(210, 0 , 255);

	}

	@Override
	public void autonomous() {

	}

	
	@Override
	public void operatorControl() {

		while (isEnabled() && isOperatorControl()) {try { Thread.sleep(10);} catch (Exception ex) {} }
	}


	@Override
	public void test() {
	}
}

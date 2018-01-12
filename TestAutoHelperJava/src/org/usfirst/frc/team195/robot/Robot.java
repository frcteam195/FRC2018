/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team195.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the RobotDrive class. The
 * SampleRobot class is the base of a robot application that will automatically
 * call your Autonomous and OperatorControl methods at the right time as
 * controlled by the switches on the driver station or the field controls.
 *
 * <p>The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 *
 * <p>WARNING: While it may look like a good choice to use for your code if
 * you're inexperienced, don't. Unless you know what you are doing, complex code
 * will be much more difficult under this system. Use IterativeRobot or
 * Command-Based instead if you're new.
 */
public class Robot extends SampleRobot {
	private Spark s0 = new Spark(0);
	private Spark s1 = new Spark(1);
	private DifferentialDrive m_robotDrive = new DifferentialDrive(s0, s1);
	private Joystick m_stick = new Joystick(0);
	private SendableChooser<String> m_chooser = new SendableChooser<>();

	private CKAutoBuilder<Spark> ckAuto;
	
	public Robot() {
		m_robotDrive.setExpiration(0.1);
		
		try {
			ckAuto = new CKAutoBuilder<Spark>(s0, s1, this);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	@Override
	public void robotInit() {

	}


	@Override
	public void autonomous() {
		ckAuto.addAutoStep(0, 0, 3000);
		ckAuto.addAutoStep(1, 0, 5000);
		ckAuto.addAutoStep(1, 0.25, 2000);
		ckAuto.addAutoStep(0, 0, 200);
		ckAuto.start();
		while(!isOperatorControl() && isEnabled()) {
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				;
			}
		}
	}

	@Override
	public void operatorControl() {
		m_robotDrive.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled()) {
			// Drive arcade style
			m_robotDrive.arcadeDrive(-m_stick.getY(), m_stick.getX());

			// The motors will be updated every 5ms
			Timer.delay(0.005);
		}
	}

	@Override
	public void test() {
	}
}

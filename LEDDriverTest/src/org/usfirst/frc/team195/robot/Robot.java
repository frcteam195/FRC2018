package org.usfirst.frc.team195.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends SampleRobot {
	private TuneablePID testTuner;
	private TalonSRX t = new TalonSRX(1);

	public Robot() {

	}

	@Override
	public void robotInit() {
		try {
			testTuner = new TuneablePID("Test", t, null, 5808, true, true);
			testTuner.start();
		} catch (Exception ex) {

		}

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

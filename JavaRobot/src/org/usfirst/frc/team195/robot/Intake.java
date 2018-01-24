package org.usfirst.frc.team195.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake {

	private Joystick driveStick;
	private Solenoid midSolenoid1;
	private Solenoid midSolenoid2;
	private TalonSRX auxRight;
	private TalonSRX auxLeft;

	public Intake() {
		driveStick = new Joystick(0);
		midSolenoid1 = new Solenoid(0);
		midSolenoid2 = new Solenoid(1);
		auxLeft = new TalonSRX(7);
		auxRight = new TalonSRX(8);

	}

	public void run() {

		if (driveStick.getRawButton(1)) {
			midSolenoid1.set(true);
			midSolenoid2.set(true);
		}

		if (driveStick.getRawButton(2)) {
			midSolenoid1.set(false);
			midSolenoid2.set(true);
		}
		
		if (driveStick.getRawButton(3)) {
			midSolenoid1.set(true);
			midSolenoid2.set(false);
		}

		if (driveStick.getRawButton(5) || driveStick.getRawButton(1)) {
			auxLeft.set(ControlMode.PercentOutput, 1);
			auxRight.set(ControlMode.PercentOutput, -1);

		} else if (driveStick.getRawButton(6)) {
			auxLeft.set(ControlMode.PercentOutput, -1);
			auxRight.set(ControlMode.PercentOutput, 1);

		} else {
			auxLeft.set(ControlMode.PercentOutput, 0);
			auxRight.set(ControlMode.PercentOutput, 0);
		}
	}
}

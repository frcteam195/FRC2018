package org.usfirst.frc.team195.robot.Utilities;

public class QuickMaths {


	public static double normalizeJoystickWithDeadband(double val, double deadband) {
		val = (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;

		if (val != 0)
			val = Math.signum(val) * ((Math.abs(val) - deadband) / (1.0 - deadband));

		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}

	public static double convertAngleToSRX(double angle, double countsPerRev) {
		return angle * countsPerRev / 360;
	}
}

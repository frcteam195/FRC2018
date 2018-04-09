package org.usfirst.frc.team195.robot.utils;

/**
 * Class to contain math functions that are used often
 * #BigShaq
 */
public class QuickMaths {


	/**
	 * Normalize joystick values (linear remapping to remove jerkiness)
	 * @param val	The joystick value to be normalized (raw input)
	 * @param deadband	The desired deadband for the joystick
	 * @return The normalized value accounting for deadband
	 */
	public static double normalizeJoystickWithDeadband(double val, double deadband) {
		val = (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;

		if (val != 0)
			val = Math.signum(val) * ((Math.abs(val) - deadband) / (1.0 - deadband));

		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}

	/**
	 * Limits the given input to the given magnitude.
	 * @param v	Value to limit
	 * @param maxMagnitude	Max magnitude for the value
	 * @return	The limited value
	 */
	public static double limit(double v, double maxMagnitude) {
		return limit(v, -maxMagnitude, maxMagnitude);
	}

	/**
	 * Limits the given input to the given min and max
	 * @param v	Value to limit
	 * @param min	Min number for the value
	 * @param max	Max number for the value
	 * @return	The limited value
	 */
	public static double limit(double v, double min, double max) {
		return Math.min(max, Math.max(min, v));
	}

	public static double convertAngleToSRX(double angle, double countsPerRev) {
		return angle * countsPerRev / 360;
	}

	public static int convertNativeUnitsToRotations(double nativeUnitsPos) {
		return (int)(nativeUnitsPos / 4096.0);
	}
}

package org.usfirst.frc.team195.robot;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;

public class Controllers {
	private static Controllers ourInstance = new Controllers();

	public static Controllers getInstance() {
		return ourInstance;
	}

	private DigitalOutput rLED;
	private DigitalOutput gLED;
	private DigitalOutput bLED;

	private Controllers() {
		rLED = new DigitalOutput(0);
		gLED = new DigitalOutput(1);
		bLED = new DigitalOutput(2);
	}

	public DigitalOutput getRedLED() {
		return rLED;
	}

	public DigitalOutput getGreenLED() {
		return gLED;
	}

	public DigitalOutput getBlueLED() {
		return bLED;
	}
}

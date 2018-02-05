package org.usfirst.frc.team195.robot.Utilities.Drivers;

import edu.wpi.first.wpilibj.DigitalOutput;
import org.usfirst.frc.team195.robot.Utilities.RGBColor;

public class LEDDriverRGB {
	private static final int PWM_FREQ = 500;

	private DigitalOutput rLED;
	private DigitalOutput gLED;
	private DigitalOutput bLED;

	private int redPWMOut = 255;
	private int greenPWMOut = 255;
	private int bluePWMOut = 255;

	public LEDDriverRGB(DigitalOutput rLED, DigitalOutput gLED, DigitalOutput bLED) {
		this.rLED = rLED;
		this.gLED = gLED;
		this.bLED = bLED;

		this.rLED.setPWMRate(PWM_FREQ);
		this.gLED.setPWMRate(PWM_FREQ);
		this.bLED.setPWMRate(PWM_FREQ);
		this.rLED.enablePWM(0);
		this.gLED.enablePWM(0);
		this.bLED.enablePWM(0);
	}

	public synchronized void set(boolean on) {
		if (on) {
			rLED.updateDutyCycle(redPWMOut/255.0);
			gLED.updateDutyCycle(greenPWMOut/255.0);
			bLED.updateDutyCycle(bluePWMOut/255.0);
		} else {
			rLED.updateDutyCycle(0);
			gLED.updateDutyCycle(0);
			bLED.updateDutyCycle(0);
		}
	}

	public synchronized void setLEDColor(RGBColor rgbColor) {
		setLEDColor(rgbColor.red, rgbColor.green, rgbColor.blue);
	}

	public synchronized void setLEDColor(int redPWMOut, int greenPWMOut, int bluePWMOut) {
		this.redPWMOut = redPWMOut;
		this.greenPWMOut = greenPWMOut;
		this.bluePWMOut = bluePWMOut;
	}

}

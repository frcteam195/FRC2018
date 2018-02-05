package org.usfirst.frc.team195.robot;

import edu.wpi.first.wpilibj.DigitalOutput;

import javax.annotation.PostConstruct;

public class LEDDriverRGB extends Thread {
	private static final int PWM_FREQ = 200;

	private static LEDDriverRGB instance = null;

	private DigitalOutput rLED;
	private DigitalOutput gLED;
	private DigitalOutput bLED;

	private boolean runThread;
	private ThreadRateControl timeControl = new ThreadRateControl();

	private int redPWMOut, greenPWMOut, bluePWMOut;

	public static LEDDriverRGB getInstance() {
		if (instance == null)
			instance = new LEDDriverRGB();

		return instance;
	}

	private LEDDriverRGB() {
		rLED = Controllers.getInstance().getRedLED();
		gLED = Controllers.getInstance().getGreenLED();
		bLED = Controllers.getInstance().getBlueLED();

		rLED.setPWMRate(500);
		gLED.setPWMRate(500);
		bLED.setPWMRate(500);
		rLED.enablePWM(0);
		gLED.enablePWM(0);
		bLED.enablePWM(0);

		runThread = false;
	}

	@Override
	public void start() {
		runThread = true;
		super.start();
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

	@Override
	public void run() {
		timeControl.start();
		while (runThread) {
			rLED.updateDutyCycle(redPWMOut/255.0);
			gLED.updateDutyCycle(greenPWMOut/255.0);
			bLED.updateDutyCycle(bluePWMOut/255.0);

			timeControl.doRateControl(20);
		}
	}

	public synchronized void setLEDColor(int redPWMOut, int greenPWMOut, int bluePWMOut) {
		this.redPWMOut = redPWMOut;
		this.greenPWMOut = greenPWMOut;
		this.bluePWMOut = bluePWMOut;
	}

}

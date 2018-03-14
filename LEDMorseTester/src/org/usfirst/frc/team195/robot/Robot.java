package org.usfirst.frc.team195.robot;


import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.ThreadRateControl;

public class Robot extends RobbieRobot {

	private LEDController ledController;
	private ThreadRateControl threadRateControl = new ThreadRateControl();

	public Robot() {
	}

	@Override
	public void robotInit() {
		ConsoleReporter.getInstance().start();
		ConsoleReporter.setReportingLevel(MessageLevel.INFO);

		ledController = LEDController.getInstance();
		ledController.start();
		ledController.setLEDColor(128, 128, 128);
		ledController.setLEDDefaultState(LEDController.LEDState.FIXED_ON);
		ledController.configureBlink(LEDController.kDefaultBlinkCount, LEDController.kDefaultBlinkDuration);
	}

	@Override
	public void disabled() {
		threadRateControl.start(true);
		while (isDisabled()) {
			ledController.setMessage("sos", true);
			threadRateControl.doRateControl(100);
		}
	}

	@Override
	public void autonomous() {

	}

	@Override
	public void operatorControl() {

	}

	@Override
	public void test() {
	}

}

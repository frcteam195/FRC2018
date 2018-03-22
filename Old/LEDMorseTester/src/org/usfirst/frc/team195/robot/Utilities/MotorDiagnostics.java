package org.usfirst.frc.team195.robot.Utilities;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Util;

public class MotorDiagnostics {
	private static final String TEST_NOT_RUN_STR = "Test not run yet!";

	private double mTestDurationSeconds = 3.0;
	private double mMotorSpeed = 0.75;
	private BaseMotorController testingSpeedController;
	private BaseMotorController masterSC = null;
	private String motorName;
	private double motorRPM;
	private double motorCurrent;
	private boolean inverted = false;
	private boolean sensorInPhase = false;
	private boolean testCompleted = false;

	public MotorDiagnostics(String motorName, BaseMotorController testingSpeedController) {
		this.testingSpeedController = testingSpeedController;
		this.motorName = motorName;
	}

	public MotorDiagnostics(String motorName, BaseMotorController testingSpeedController, double motorSpeed) {
		this(motorName, testingSpeedController);
		this.mMotorSpeed = motorSpeed;
	}

	public MotorDiagnostics(String motorName, BaseMotorController testingSpeedController, double motorSpeed, double testDurationSeconds, boolean inverted) {
		this(motorName, testingSpeedController, motorSpeed);
		this.inverted = inverted;
		this.mTestDurationSeconds = testDurationSeconds;
	}

	public MotorDiagnostics(String motorName, BaseMotorController testingSpeedController, BaseMotorController masterSC) {
		this(motorName, testingSpeedController);
		this.masterSC = masterSC;
	}

	public MotorDiagnostics(String motorName, BaseMotorController testingSpeedController, BaseMotorController masterSC, double motorSpeed) {
		this(motorName, testingSpeedController, motorSpeed);
		this.masterSC = masterSC;
	}

	public MotorDiagnostics(String motorName, BaseMotorController testingSpeedController, BaseMotorController masterSC, double motorSpeed, double testDurationSeconds, boolean inverted) {
		this(motorName, testingSpeedController, motorSpeed, testDurationSeconds, inverted);
		this.masterSC = masterSC;
	}

	public void runTest() {
		testingSpeedController.configForwardSoftLimitEnable(false, Constants.kTimeoutMs);
		testingSpeedController.configReverseSoftLimitEnable(false, Constants.kTimeoutMs);

		double motorPositionPreTest = getPosition();
		mMotorSpeed = Math.abs(mMotorSpeed);
		testingSpeedController.set(ControlMode.PercentOutput, inverted ? -mMotorSpeed : mMotorSpeed);
		Timer.delay(mTestDurationSeconds/2.0);
		motorCurrent = testingSpeedController.getOutputCurrent();
		motorRPM = getRPM();
		double motorPositionPostTest = getPosition();
		Timer.delay(mTestDurationSeconds/2.0);
		testingSpeedController.set(ControlMode.PercentOutput, 0);

		if (inverted & (motorPositionPostTest < motorPositionPreTest))
			sensorInPhase =  true;
		else if (!inverted & (motorPositionPostTest > motorPositionPreTest))
			sensorInPhase = true;
		else
			sensorInPhase = false;

		testCompleted = true;
	}

	public boolean isSensorInPhase() {
		if (testCompleted)
			return sensorInPhase;
		else {
			ConsoleReporter.report(TEST_NOT_RUN_STR, MessageLevel.ERROR);
			return false;
		}
	}

	private double getPosition() {
		return QuickMaths.convertNativeUnitsToRotations(masterSC == null ? testingSpeedController.getSelectedSensorPosition(0)
				: masterSC.getSelectedSensorPosition(0));
	}

	private double getRPM() {
		return Util.convertNativeUnitsToRPM(masterSC == null ? testingSpeedController.getSelectedSensorVelocity(0)
			: masterSC.getSelectedSensorVelocity(0));
	}

	public void setZero() {
		testingSpeedController.set(ControlMode.PercentOutput, 0);
	}

	public String getMotorName() {
		return motorName;
	}

	public double getMotorCurrent() {
		if (testCompleted)
			return motorCurrent;
		else {
			ConsoleReporter.report(TEST_NOT_RUN_STR, MessageLevel.ERROR);
			return 0;
		}
	}

	public double getMotorRPM() {
		if (testCompleted)
			return motorRPM;
		else {
			ConsoleReporter.report(TEST_NOT_RUN_STR, MessageLevel.ERROR);
			return 0;
		}
	}

	public boolean isCurrentUnderThreshold(double threshold) {
		if (testCompleted)
			return motorCurrent < threshold;
		else {
			ConsoleReporter.report(TEST_NOT_RUN_STR, MessageLevel.ERROR);
			return true;
		}
	}

	public boolean isRPMUnderThreshold(double threshold) {
		if (testCompleted)
			return motorRPM < threshold;
		else {
			ConsoleReporter.report(TEST_NOT_RUN_STR, MessageLevel.ERROR);
			return true;
		}
	}

	@Override
	public String toString() {
		String retVal = "";
		retVal += motorName + "\r\n";
		retVal += "\tCurrent: " + motorCurrent + "\r\n";
		retVal += "\tRPM: " + motorRPM + "\r\n";
		return retVal;
	}

}

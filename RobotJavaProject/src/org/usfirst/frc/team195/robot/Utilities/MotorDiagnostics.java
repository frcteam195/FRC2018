package org.usfirst.frc.team195.robot.Utilities;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Util;

public class MotorDiagnostics {
	private double mTestDurationSeconds = 4.0;
	private double mMotorSpeed = 0.5;
	private BaseMotorController testingSpeedController;
	private BaseMotorController masterSC = null;
	private String motorName;
	private double motorRPM;
	private double motorCurrent;
	private boolean inverted = false;

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
		mMotorSpeed = Math.abs(mMotorSpeed);
		testingSpeedController.set(ControlMode.PercentOutput, inverted ? -mMotorSpeed : mMotorSpeed);
		Timer.delay(mTestDurationSeconds/2.0);
		motorCurrent = testingSpeedController.getOutputCurrent();
		motorRPM =  Util.convertNativeUnitsToRPM(masterSC == null ? testingSpeedController.getSelectedSensorVelocity(0)
				: masterSC.getSelectedSensorVelocity(0));
		Timer.delay(mTestDurationSeconds/2.0);
		testingSpeedController.set(ControlMode.PercentOutput, 0);
	}

	public void setZero() {
		testingSpeedController.set(ControlMode.PercentOutput, 0);
	}

	public String getMotorName() {
		return motorName;
	}

	public double getMotorCurrent() {
		return motorCurrent;
	}

	public double getMotorRPM() {
		return motorRPM;
	}

	public boolean isCurrentUnderThreshold(double threshold) {
		return motorCurrent < threshold;
	}

	public boolean isCurrentOverThreshold(double threshold) {
		return motorCurrent > threshold;
	}

	public boolean isRPMUnderThreshold(double threshold) {
		return motorRPM < threshold;
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

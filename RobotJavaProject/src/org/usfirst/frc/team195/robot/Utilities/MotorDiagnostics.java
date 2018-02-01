package org.usfirst.frc.team195.robot.Utilities;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.wpilibj.Timer;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Util;

public class MotorDiagnostics {
	private static final double kTestDurationSeconds = 4.0;
	private BaseMotorController testingSpeedController;
	private BaseMotorController masterSC = null;
	private String motorName;
	private double motorRPM;
	private double motorCurrent;

	public MotorDiagnostics(String motorName, BaseMotorController testingSpeedController) {
		this.testingSpeedController = testingSpeedController;
		this.motorName = motorName;
	}

	public MotorDiagnostics(String motorName, BaseMotorController testingSpeedController, BaseMotorController masterSC) {
		this.testingSpeedController = testingSpeedController;
		this.masterSC = masterSC;
		this.motorName = motorName;
	}

	public void runTest() {
		testingSpeedController.set(ControlMode.PercentOutput, 0.5);
		Timer.delay(kTestDurationSeconds);
		motorCurrent = testingSpeedController.getOutputCurrent();
		motorRPM =  Util.convertNativeUnitsToRPM(masterSC == null ? testingSpeedController.getSelectedSensorVelocity(0)
				: masterSC.getSelectedSensorVelocity(0));
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

}

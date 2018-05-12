package org.usfirst.frc.team195.robot.Utilities.Drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.Constants;

public class PigeonDriver {
	private PigeonIMU pigeonIMU;

	public PigeonDriver(int deviceNumber) {
		pigeonIMU = new PigeonIMU(deviceNumber);
	}

	public boolean reset() {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = pigeonIMU.setYaw(0, Constants.kTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to reset Pigeon " + pigeonIMU.getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);

		return setSucceeded;
	}
}

package org.usfirst.frc.team195.robot.Utilities.Drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.usfirst.frc.team195.robot.Utilities.Constants;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Util;

public class TalonHelper {
	public static boolean setPIDGains(TalonSRX talon, int slotID, double kP, double kI, double kD, double kF) {
		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			setSucceeded &= talon.config_kP(slotID, kP, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= talon.config_kI(slotID, kI, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= talon.config_kD(slotID, kD, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= talon.config_kF(slotID, kF, Constants.kTimeoutMs) == ErrorCode.OK;

		} while (!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		return retryCounter < Constants.kTalonRetryCount && setSucceeded;
	}

	public static boolean setPIDGains(TalonSRX talon, int slotID, double kP, double kI, double kD, double kF, double rampRate, int iZone) {
		boolean setSucceeded = setPIDGains(talon, slotID, kP, kI, kD, kF);
		int retryCounter = 0;

		do {
			setSucceeded = true;

			setSucceeded &= talon.configClosedloopRamp(rampRate, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= talon.config_IntegralZone(slotID, iZone, Constants.kTimeoutMs) == ErrorCode.OK;

		} while (!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		return retryCounter < Constants.kTalonRetryCount && setSucceeded;
	}

	public static boolean setMotionMagicParams(TalonSRX talon, int cruiseVelocityRPM, int maxAccelRPM) {
		boolean setSucceeded = true;
		int retryCounter = 0;

		do {
			setSucceeded = true;

			setSucceeded &= talon.configMotionCruiseVelocity(Util.convertRPMToNativeUnits(cruiseVelocityRPM), Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= talon.configMotionAcceleration(Util.convertRPMToNativeUnits(maxAccelRPM), Constants.kTimeoutMs) == ErrorCode.OK;

		} while (!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		return retryCounter < Constants.kTalonRetryCount && setSucceeded;
	}
}

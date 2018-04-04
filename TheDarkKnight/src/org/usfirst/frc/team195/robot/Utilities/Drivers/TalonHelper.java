package org.usfirst.frc.team195.robot.Utilities.Drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.usfirst.frc.team195.robot.Utilities.Constants;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.Util;

public class TalonHelper {
	public static boolean setPIDGains(TalonSRX talon, int slotID, double kP, double kI, double kD, double kF) {
		return setPIDGains(talon, slotID, kP, kI, kD, kF, Constants.kTimeoutMs);
	}
	public static boolean setPIDGains(TalonSRX talon, int slotID, double kP, double kI, double kD, double kF, int timeout) {
		boolean setSucceeded = true;
		int retryCounter = 0;
		if (timeout > 0) {
			do {
				setSucceeded = true;

				setSucceeded &= talon.config_kP(slotID, kP, timeout) == ErrorCode.OK;
				setSucceeded &= talon.config_kI(slotID, kI, timeout) == ErrorCode.OK;
				setSucceeded &= talon.config_kD(slotID, kD, timeout) == ErrorCode.OK;
				setSucceeded &= talon.config_kF(slotID, kF, timeout) == ErrorCode.OK;

			} while (!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);
		} else {
			talon.config_kP(slotID, kP, timeout);
			talon.config_kI(slotID, kI, timeout);
			talon.config_kD(slotID, kD, timeout);
			talon.config_kF(slotID, kF, timeout);
		}
		return retryCounter < Constants.kTalonRetryCount && setSucceeded;
	}

	public static boolean setPIDGains(TalonSRX talon, int slotID, double kP, double kI, double kD, double kF, double rampRate, int iZone) {
		return setPIDGains(talon, slotID, kP, kI, kD, kF, rampRate, iZone, Constants.kTimeoutMs);
	}
	public static boolean setPIDGains(TalonSRX talon, int slotID, double kP, double kI, double kD, double kF, double rampRate, int iZone, int timeout) {
		boolean setSucceeded = setPIDGains(talon, slotID, kP, kI, kD, kF, timeout);
		int retryCounter = 0;

		if (timeout > 0) {
			do {
				setSucceeded = true;

				setSucceeded &= talon.configClosedloopRamp(rampRate, timeout) == ErrorCode.OK;
				setSucceeded &= talon.config_IntegralZone(slotID, iZone, timeout) == ErrorCode.OK;

			} while (!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);
		} else {
			talon.configClosedloopRamp(rampRate, timeout);
			talon.config_IntegralZone(slotID, iZone, timeout);
		}
		return retryCounter < Constants.kTalonRetryCount && setSucceeded;
	}

	public static boolean setPIDGains(CKTalonSRX talon, int slotID, double kP, double kI, double kD, double kF, double rampRate, int iZone) {
		return setPIDGains(talon, slotID, kP, kI, kD, kF, rampRate, iZone, Constants.kTimeoutMs);
	}
	public static boolean setPIDGains(CKTalonSRX talon, int slotID, double kP, double kI, double kD, double kF, double rampRate, int iZone, int timeout) {
		boolean setSucceeded = setPIDGains(talon, slotID, kP, kI, kD, kF, timeout);
		int retryCounter = 0;

		if (timeout > 0) {
			do {
				setSucceeded = true;

				setSucceeded &= talon.configClosedloopRamp(rampRate, slotID, timeout) == ErrorCode.OK;
				setSucceeded &= talon.config_IntegralZone(slotID, iZone, timeout) == ErrorCode.OK;

			} while (!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);
		} else {
			talon.configClosedloopRamp(rampRate, slotID, timeout);
			talon.config_IntegralZone(slotID, iZone, timeout);
		}
		return retryCounter < Constants.kTalonRetryCount && setSucceeded;
	}

	public static boolean setMotionMagicParams(TalonSRX talon, int cruiseVelocityRPM, int maxAccelRPM) {
		return setMotionMagicParams(talon, cruiseVelocityRPM, maxAccelRPM, Constants.kTimeoutMs);
	}
	public static boolean setMotionMagicParams(TalonSRX talon, int cruiseVelocityRPM, int maxAccelRPM, int timeout) {
		boolean setSucceeded = true;
		int retryCounter = 0;
		if (timeout > 0) {
			do {
				setSucceeded = true;

				setSucceeded &= talon.configMotionCruiseVelocity(Util.convertRPMToNativeUnits(cruiseVelocityRPM), timeout) == ErrorCode.OK;
				setSucceeded &= talon.configMotionAcceleration(Util.convertRPMToNativeUnits(maxAccelRPM), timeout) == ErrorCode.OK;

			} while (!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);
		} else {
			talon.configMotionCruiseVelocity(Util.convertRPMToNativeUnits(cruiseVelocityRPM), timeout);
			talon.configMotionAcceleration(Util.convertRPMToNativeUnits(maxAccelRPM), timeout);
		}
		return retryCounter < Constants.kTalonRetryCount && setSucceeded;
	}
}

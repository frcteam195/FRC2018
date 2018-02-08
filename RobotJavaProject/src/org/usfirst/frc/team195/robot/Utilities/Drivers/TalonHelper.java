package org.usfirst.frc.team195.robot.Utilities.Drivers;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.usfirst.frc.team195.robot.Utilities.Constants;

public class TalonHelper {
	public static void setPIDGains(TalonSRX talon, int slotID, double kP, double kI, double kD, double kF) {
		talon.config_kP(slotID, kP, Constants.kTimeoutMs);
		talon.config_kI(slotID, kI, Constants.kTimeoutMs);
		talon.config_kD(slotID, kD, Constants.kTimeoutMs);
		talon.config_kF(slotID, kF, Constants.kTimeoutMs);
	}

	public static void setPIDGains(TalonSRX talon, int slotID, double kP, double kI, double kD, double kF, double rampRate, int iZone) {
		setPIDGains(talon, slotID, kP, kI, kD, kF);
		talon.configClosedloopRamp(rampRate, Constants.kTimeoutMs);
		talon.config_IntegralZone(slotID, iZone, Constants.kTimeoutMs);
	}
}

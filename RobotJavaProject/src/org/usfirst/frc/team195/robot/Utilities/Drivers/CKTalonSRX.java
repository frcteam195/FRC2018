package org.usfirst.frc.team195.robot.Utilities.Drivers;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.usfirst.frc.team195.robot.Utilities.Controllers;

public class CKTalonSRX extends TalonSRX {
	int pdpChannel;

	public CKTalonSRX(int deviceId, int pdpChannel) {
		super(deviceId);
		this.pdpChannel = pdpChannel;
	}

	public double getPDPCurrent() {
		return Controllers.getInstance().getPowerDistributionPanel().getCurrent(pdpChannel);
	}
}

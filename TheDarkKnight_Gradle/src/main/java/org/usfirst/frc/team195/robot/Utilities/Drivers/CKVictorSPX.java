package org.usfirst.frc.team195.robot.Utilities.Drivers;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import org.usfirst.frc.team195.robot.Utilities.Controllers;

public class CKVictorSPX extends VictorSPX {
	int pdpChannel;

	public CKVictorSPX(int deviceId, int pdpChannel) {
		super(deviceId);
		this.pdpChannel = pdpChannel;
	}

	@Override
	public double getOutputCurrent() {
		return Controllers.getInstance().getPowerDistributionPanel().getCurrent(pdpChannel);
	}
}

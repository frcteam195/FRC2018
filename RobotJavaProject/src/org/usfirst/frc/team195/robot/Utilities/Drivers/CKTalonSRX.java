package org.usfirst.frc.team195.robot.Utilities.Drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team195.robot.Utilities.Controllers;

public class CKTalonSRX extends TalonSRX {
	int pdpChannel;

	public CKTalonSRX(int deviceId, int pdpChannel) {
		super(deviceId);
		this.pdpChannel = pdpChannel;
	}

//	@Override
//	public void set(ControlMode mode, double outputValue) {
//		super.set(mode, outputValue);
//		SmartDashboard.putNumber("OutputTalon" + getDeviceID(), outputValue);
//		SmartDashboard.putString("ModeTalon" + getDeviceID(), getControlMode().toString());
//	}

	public double getPDPCurrent() {
		return Controllers.getInstance().getPowerDistributionPanel().getCurrent(pdpChannel);
	}
}

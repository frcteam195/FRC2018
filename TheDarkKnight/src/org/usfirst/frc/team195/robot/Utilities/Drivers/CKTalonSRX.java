package org.usfirst.frc.team195.robot.Utilities.Drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team195.robot.Utilities.Constants;
import org.usfirst.frc.team195.robot.Utilities.Controllers;

public class CKTalonSRX extends TalonSRX {
	private int pdpChannel;
	private int currentSelectedSlot = 0;

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


	@Override
	public void selectProfileSlot(int slotIdx, int pidIdx) {
		super.selectProfileSlot(slotIdx, pidIdx);
		setCurrentSlotValue(slotIdx);
	}

	public void set(ControlMode mode, double outputValue, int slotIdx) {
		if (currentSelectedSlot != slotIdx)
			selectProfileSlot(slotIdx, 0);

		set(mode, outputValue);
	}

	private synchronized void setCurrentSlotValue(int slotIdx) {
		currentSelectedSlot = slotIdx;
	}

	public double getPDPCurrent() {
		return Controllers.getInstance().getPowerDistributionPanel().getCurrent(pdpChannel);
	}

	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append("General Status Frame 1: " + getStatusFramePeriod(StatusFrameEnhanced.Status_1_General, Constants.kTimeoutMs) + "\r\n");
		sb.append("General Status Frame 2: " + getStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, Constants.kTimeoutMs) + "\r\n");
		sb.append("General Status Frame 3: " + getStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, Constants.kTimeoutMs) + "\r\n");
		sb.append("General Status Frame 4: " + getStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, Constants.kTimeoutMs) + "\r\n");
		sb.append("General Status Frame 6: " + getStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, Constants.kTimeoutMs) + "\r\n");
		sb.append("General Status Frame 7: " + getStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, Constants.kTimeoutMs) + "\r\n");
		sb.append("General Status Frame 8: " + getStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, Constants.kTimeoutMs) + "\r\n");
		sb.append("General Status Frame 9: " + getStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, Constants.kTimeoutMs) + "\r\n");
		sb.append("General Status Frame 10: " + getStatusFramePeriod(StatusFrame.Status_10_Targets, Constants.kTimeoutMs) + "\r\n");
		sb.append("General Status Frame 11: " + getStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, Constants.kTimeoutMs) + "\r\n");
		sb.append("General Status Frame 12: " + getStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, Constants.kTimeoutMs) + "\r\n");
		sb.append("General Status Frame 13: " + getStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, Constants.kTimeoutMs) + "\r\n");
		sb.append("General Status Frame 14: " + getStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, Constants.kTimeoutMs) + "\r\n");
		sb.append("General Status Frame 15: " + getStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmareApiStatus, Constants.kTimeoutMs) + "\r\n");
		return sb.toString();
	}
}

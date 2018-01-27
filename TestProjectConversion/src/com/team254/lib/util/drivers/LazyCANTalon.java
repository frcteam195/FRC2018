package com.team254.lib.util.drivers;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping duplicate set
 * commands. (By default the Talon flushes the Tx buffer on every set call).
 */
public class LazyCANTalon extends TalonSRX {
    protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = null;

    public LazyCANTalon(int deviceNumber, int controlPeriodMs, int enablePeriodMs) {
        super(deviceNumber);
        super.setControlFramePeriod(ControlFrame.Control_3_General, controlPeriodMs);
    }

    public LazyCANTalon(int deviceNumber, int controlPeriodMs) {
        super(deviceNumber);
        super.setControlFramePeriod(ControlFrame.Control_3_General, controlPeriodMs);
    }

    public LazyCANTalon(int deviceNumber) {
        super(deviceNumber);
    }

    public void set(double value) {
        set(getControlMode(), value);
    }

    @Override
    public void set(ControlMode controlMode, double value) {
        if (value != mLastSet || getControlMode() != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = getControlMode();
            super.set(mLastControlMode, value);
        }
    }
}

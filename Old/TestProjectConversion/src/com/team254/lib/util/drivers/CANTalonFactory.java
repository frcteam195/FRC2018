package com.team254.lib.util.drivers;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team254.frc2017.Constants;

public class CANTalonFactory {
    private static class Configuration {
        public double MAX_OUTPUT = 1;
        public double NOMINAL_OUTPUT = 0;
        public NeutralMode NEUTRAL_MODE = NeutralMode.Brake;
        public boolean ENABLE_CURRENT_LIMIT = false;
        public boolean ENABLE_SOFT_LIMIT = false;
        //public boolean ENABLE_LIMIT_SWITCH = false;
        public int CURRENT_LIMIT = 0;
        public boolean INVERTED = false;

        public int CONTROL_FRAME_PERIOD_MS = 5;
        public int GENERAL_STATUS_FRAME_RATE_MS = 5;

        //public VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = VelocityMeasPeriod::Period_100Ms;
        //public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;
    }

    private static Configuration kDefaultConfiguration = new Configuration();
    private static Configuration kSlaveConfiguration = new Configuration();

    public static TalonSRX createDefaultTalonSRX(int id) {
        return createTalonSRX(id, kDefaultConfiguration);
    }

    public static TalonSRX createPermanentSlaveTalonSRX(int id, TalonSRX masterTalon) {
        TalonSRX talon = createTalonSRX(id, kSlaveConfiguration);
        talon.follow(masterTalon);
        return talon;
    }

    public static VictorSPX createPermanentVictorSlaveToTalonSRX(int id, TalonSRX masterTalon) {
        VictorSPX victor = new VictorSPX(id);
        victor.follow(masterTalon);
        return victor;
    }

    public static TalonSRX createTalonSRX(int id, Configuration config) {
        TalonSRX talon = new TalonSRX(id);

        talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);
        talon.setStatusFramePeriod(StatusFrame.Status_1_General, config.GENERAL_STATUS_FRAME_RATE_MS, Constants.kTimeoutMs);
        talon.set(ControlMode.PercentOutput, 0);
        talon.setIntegralAccumulator(0, 0, Constants.kTimeoutMs);
        talon.clearMotionProfileHasUnderrun(Constants.kTimeoutMs);
        talon.clearMotionProfileTrajectories();
        talon.clearStickyFaults(Constants.kTimeoutMs);
        talon.configPeakOutputForward(config.MAX_OUTPUT, Constants.kTimeoutMs);
        talon.configPeakOutputReverse(-config.MAX_OUTPUT, Constants.kTimeoutMs);
        talon.configNominalOutputForward(config.NOMINAL_OUTPUT, Constants.kTimeoutMs);
        talon.configNominalOutputReverse(-config.NOMINAL_OUTPUT, Constants.kTimeoutMs);
        talon.setNeutralMode(config.NEUTRAL_MODE);
        talon.enableCurrentLimit(config.ENABLE_CURRENT_LIMIT);
        talon.configContinuousCurrentLimit(config.CURRENT_LIMIT, Constants.kTimeoutMs);
        talon.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, Constants.kTimeoutMs);
        talon.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, Constants.kTimeoutMs);
        talon.setInverted(config.INVERTED);
        talon.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs);
        //talon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, Constants.kTimeoutMs);
        //talon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, Constants.kTimeoutMs);

        return talon;
    }
}
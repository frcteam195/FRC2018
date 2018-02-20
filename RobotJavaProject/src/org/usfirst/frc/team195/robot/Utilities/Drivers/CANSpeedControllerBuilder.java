package org.usfirst.frc.team195.robot.Utilities.Drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import org.usfirst.frc.team195.robot.Reporters.ConsoleReporter;
import org.usfirst.frc.team195.robot.Reporters.MessageLevel;
import org.usfirst.frc.team195.robot.Utilities.Constants;

public class CANSpeedControllerBuilder {
	private static class Configuration {
		public double MAX_OUTPUT = 1;
		public double NOMINAL_OUTPUT = 0;
		public NeutralMode NEUTRAL_MODE = NeutralMode.Brake;
		public boolean ENABLE_CURRENT_LIMIT = false;
		public boolean ENABLE_SOFT_LIMIT = false;
		//public boolean ENABLE_LIMIT_SWITCH = false;
		public int CURRENT_LIMIT = 0;
		public boolean INVERTED = false;

		public int CONTROL_FRAME_PERIOD_MS = 10;
		public int GENERAL_STATUS_FRAME_RATE_MS = 10;

		//public VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = VelocityMeasPeriod::Period_100Ms;
		//public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;
	}
	
	private static Configuration kDefaultConfiguration = new Configuration();
	private static Configuration kSlaveConfiguration = new Configuration();
	
	private CANSpeedControllerBuilder() { }
	
	public static TalonSRX createDefaultTalonSRX(int id) {
        return createTalonSRX(id, kDefaultConfiguration);
    }

	public static CKTalonSRX createDefaultTalonSRX(int id, int pdpChannel) {
		return createTalonSRX(id, pdpChannel, kDefaultConfiguration);
	}

	public static TalonSRX createMasterTalonSRX(int id) {
		Configuration masterConfig = new Configuration();
		return createTalonSRX(id, masterConfig);
	}

	public static CKTalonSRX createMasterTalonSRX(int id, int pdpChannel) {
		Configuration masterConfig = new Configuration();
		return createTalonSRX(id, pdpChannel, masterConfig);
	}
	
	public static TalonSRX createPermanentSlaveTalonSRX(int id, TalonSRX masterTalon) {
		TalonSRX talon = createTalonSRX(id, kSlaveConfiguration);
		talon.follow(masterTalon);
		return talon;
	}

	public static CKTalonSRX createPermanentSlaveTalonSRX(int id, int pdpChannel, TalonSRX masterTalon) {
		CKTalonSRX talon = createTalonSRX(id, pdpChannel, kSlaveConfiguration);
		talon.follow(masterTalon);
		return talon;
	}

	@Deprecated
	public static VictorSPX createPermanentVictorSlaveToTalonSRX(int id, TalonSRX masterTalon) {
		VictorSPX victor = new VictorSPX(id);
		victor.follow(masterTalon);
		return victor;
	}

	public static CKVictorSPX createPermanentVictorSlaveToTalonSRX(int id, int pdpChannel, TalonSRX masterTalon) {
		CKVictorSPX victor = new CKVictorSPX(id, pdpChannel);
		victor.follow(masterTalon);
		return victor;
	}

	public static TalonSRX createTalonSRX(int id, Configuration config) {
        TalonSRX talon = new TalonSRX(id);
        //TODO: Test to make sure CAN utilization is not too high
        //configTalon(talon, config);
        return talon;
    }

	public static CKTalonSRX createTalonSRX(int id, int pdpChannel, Configuration config) {
		CKTalonSRX talon = new CKTalonSRX(id, pdpChannel);
		//configTalon(talon, config);
		return talon;
	}

	private static boolean configTalon(TalonSRX talon, Configuration config) {
		talon.set(ControlMode.PercentOutput, 0);

		talon.setNeutralMode(config.NEUTRAL_MODE);
		talon.setInverted(config.INVERTED);
		talon.enableCurrentLimit(config.ENABLE_CURRENT_LIMIT);
		talon.clearMotionProfileTrajectories();

		boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;
			setSucceeded &= talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS) == ErrorCode.OK;
			setSucceeded &= talon.setStatusFramePeriod(StatusFrame.Status_1_General, config.GENERAL_STATUS_FRAME_RATE_MS, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= talon.setIntegralAccumulator(0, 0, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= talon.clearMotionProfileHasUnderrun(Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= talon.clearStickyFaults(Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= talon.configPeakOutputForward(config.MAX_OUTPUT, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= talon.configPeakOutputReverse(-config.MAX_OUTPUT, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= talon.configNominalOutputForward(config.NOMINAL_OUTPUT, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= talon.configNominalOutputReverse(-config.NOMINAL_OUTPUT, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= talon.configContinuousCurrentLimit(config.CURRENT_LIMIT, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= talon.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= talon.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, Constants.kTimeoutMs) == ErrorCode.OK;
			setSucceeded &= talon.setSelectedSensorPosition(0, 0, Constants.kTimeoutMs) == ErrorCode.OK;
			//setSucceeded &= talon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, Constants.kTimeoutMs) == ErrorCode.OK;
			//setSucceeded &= talon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, Constants.kTimeoutMs) == ErrorCode.OK;
		} while(!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);

		if (retryCounter >= Constants.kTalonRetryCount || !setSucceeded)
			ConsoleReporter.report("Failed to initialize Talon " + talon.getDeviceID() + " !!!!!!", MessageLevel.DEFCON1);

		return setSucceeded;
	}
}
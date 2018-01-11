/*
 * TalonSRXBuilder.h
 *
 *  Created on: Jan 11, 2018
 *      Author: roberthilton
 */

#ifndef SRC_UTILITIES_TALONSRXBUILDER_H_
#define SRC_UTILITIES_TALONSRXBUILDER_H_


#include "WPILib.h"
#include "Utilities/Constants.h"
#include "ctre/Phoenix.h"

using namespace std;
using namespace frc;

class Configuration {
public:
	double MAX_OUTPUT = 1;
	double NOMINAL_OUTPUT = 0;
	NeutralMode NEUTRAL_MODE = NeutralMode::Coast;
	bool ENABLE_CURRENT_LIMIT = false;
	bool ENABLE_SOFT_LIMIT = false;
	bool ENABLE_LIMIT_SWITCH = false;
	int CURRENT_LIMIT = 0;
	bool INVERTED = false;

	int CONTROL_FRAME_PERIOD_MS = 5;
	int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
	int GENERAL_STATUS_FRAME_RATE_MS = 5;
	int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
	int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 100;
	int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 100;
	int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 100;

	VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = VelocityMeasPeriod::Period_100Ms;
	int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;
};

class TalonSRXBuilder {
private:
	Configuration *kDefaultConfiguration;
	Configuration *kSlaveConfiguration;


public:
    TalonSRXBuilder() {
    		kDefaultConfiguration = new Configuration();

    		kSlaveConfiguration = new Configuration();
    		kSlaveConfiguration->CONTROL_FRAME_PERIOD_MS = 1000;
		kSlaveConfiguration->MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
		kSlaveConfiguration->GENERAL_STATUS_FRAME_RATE_MS = 1000;
		kSlaveConfiguration->FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
		kSlaveConfiguration->QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
		kSlaveConfiguration->ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
		kSlaveConfiguration->PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
	};
	~TalonSRXBuilder() {};

    TalonSRX *createDefaultTalonSRX(int id) {
        return createTalonSRX(id, kDefaultConfiguration);
    };

    TalonSRX *createPermanentSlaveTalonSRX(int id, TalonSRX *masterTalon) {
    		return createPermanentSlaveTalonSRX(id, masterTalon->GetDeviceID());
    }

    TalonSRX *createPermanentSlaveTalonSRX(int id, int master_id) {
        TalonSRX *talon = createTalonSRX(id, kSlaveConfiguration);

        talon->Set(ControlMode::Follower, master_id);
        talon->SetControlFramePeriod(ControlFrame::Control_3_General, kSlaveConfiguration->CONTROL_FRAME_PERIOD_MS);
		talon->ChangeMotionControlFramePeriod(kSlaveConfiguration->MOTION_CONTROL_FRAME_PERIOD_MS);
		talon->SetStatusFramePeriod(StatusFrame::Status_1_General_, kSlaveConfiguration->GENERAL_STATUS_FRAME_RATE_MS, kTimeoutMs);
		talon->SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, kSlaveConfiguration->GENERAL_STATUS_FRAME_RATE_MS, kTimeoutMs);
		talon->SetStatusFramePeriod(StatusFrame::Status_4_AinTempVbat_, kSlaveConfiguration->GENERAL_STATUS_FRAME_RATE_MS, kTimeoutMs);
		talon->SetStatusFramePeriod(StatusFrame::Status_6_Misc_, kSlaveConfiguration->GENERAL_STATUS_FRAME_RATE_MS, kTimeoutMs);

        return talon;
    };

	TalonSRX *createTalonSRX(int id, Configuration *config) {
        TalonSRX *talon = new TalonSRX(id);

        talon->SetControlFramePeriod(ControlFrame::Control_3_General, config->CONTROL_FRAME_PERIOD_MS);
        talon->ChangeMotionControlFramePeriod(config->MOTION_CONTROL_FRAME_PERIOD_MS);
        talon->SetStatusFramePeriod(StatusFrame::Status_1_General_, config->GENERAL_STATUS_FRAME_RATE_MS, kTimeoutMs);
        talon->Set(ControlMode::PercentOutput, 0);
        talon->SetIntegralAccumulator(0, 0, kTimeoutMs);
        talon->ClearMotionProfileHasUnderrun(kTimeoutMs);
        talon->ClearMotionProfileTrajectories();
        talon->ClearStickyFaults(kTimeoutMs);
        talon->ConfigPeakOutputForward(config->MAX_OUTPUT, kTimeoutMs);
        talon->ConfigPeakOutputReverse(-config->MAX_OUTPUT, kTimeoutMs);
        talon->ConfigNominalOutputForward(config->NOMINAL_OUTPUT, kTimeoutMs);
        talon->ConfigNominalOutputReverse(-config->NOMINAL_OUTPUT, kTimeoutMs);
        talon->SetNeutralMode(config->NEUTRAL_MODE);
        talon->EnableCurrentLimit(config->ENABLE_CURRENT_LIMIT);
        talon->ConfigContinuousCurrentLimit(config->CURRENT_LIMIT, kTimeoutMs);
        talon->ConfigForwardSoftLimitEnable(config->ENABLE_SOFT_LIMIT, kTimeoutMs);
        talon->ConfigReverseSoftLimitEnable(config->ENABLE_SOFT_LIMIT, kTimeoutMs);
        talon->SetInverted(config->INVERTED);
        talon->SetSelectedSensorPosition(0, 0, kTimeoutMs);
        talon->ConfigVelocityMeasurementPeriod(config->VELOCITY_MEASUREMENT_PERIOD, kTimeoutMs);
        talon->ConfigVelocityMeasurementWindow(config->VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, kTimeoutMs);

        return talon;
    }

};

#endif /* SRC_UTILITIES_TALONSRXBUILDER_H_ */

/*
 * TalonSRXBuilder.h
 *
 *  Created on: Jan 11, 2018
 *      Author: roberthilton
 */

#ifndef SRC_UTILITIES_CANSPEEDCONTROLLERBUILDER_H_
#define SRC_UTILITIES_CANSPEEDCONTROLLERBUILDER_H_


#include "WPILib.h"
#include "Utilities/Constants.h"
#include "ctre/Phoenix.h"

using namespace std;
using namespace frc;

class Configuration {
public:
	double MAX_OUTPUT = 1;
	double NOMINAL_OUTPUT = 0;
	NeutralMode NEUTRAL_MODE = NeutralMode::Brake;
	bool ENABLE_CURRENT_LIMIT = false;
	bool ENABLE_SOFT_LIMIT = false;
	bool ENABLE_LIMIT_SWITCH = false;
	int CURRENT_LIMIT = 0;
	bool INVERTED = false;

	int CONTROL_FRAME_PERIOD_MS = 5;
	int GENERAL_STATUS_FRAME_RATE_MS = 5;

	//VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = VelocityMeasPeriod::Period_100Ms;
	//int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;
};

class CANSpeedControllerBuilder {
private:
	Configuration *kDefaultConfiguration;
	Configuration *kSlaveConfiguration;

public:
    CANSpeedControllerBuilder() {
    		kDefaultConfiguration = new Configuration();

    		kSlaveConfiguration = new Configuration();
	};
	~CANSpeedControllerBuilder() {};

    TalonSRX *createDefaultTalonSRX(int id) {
        return createTalonSRX(id, kDefaultConfiguration);
    };

    TalonSRX *createPermanentSlaveTalonSRX(int id, TalonSRX *masterTalon) {
    		TalonSRX *talon = createTalonSRX(id, kSlaveConfiguration);
    		talon->Follow(*masterTalon);
    		return talon;
    };

    VictorSPX* createPermanentVictorSlaveToTalonSRX(int id, TalonSRX* masterTalon) {
    		VictorSPX* victor = new VictorSPX(id);
    		victor->Follow(*masterTalon);
    		return victor;
    };

	TalonSRX *createTalonSRX(int id, Configuration *config) {
        TalonSRX *talon = new TalonSRX(id);

        talon->SetControlFramePeriod(ControlFrame::Control_3_General, config->CONTROL_FRAME_PERIOD_MS);
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
        //talon->ConfigVelocityMeasurementPeriod(config->VELOCITY_MEASUREMENT_PERIOD, kTimeoutMs);
        //talon->ConfigVelocityMeasurementWindow(config->VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, kTimeoutMs);

        return talon;
    }

};

#endif /* SRC_UTILITIES_CANSPEEDCONTROLLERBUILDER_H_ */

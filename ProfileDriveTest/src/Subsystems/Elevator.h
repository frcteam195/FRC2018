/*
 * Elevator.h
 *
 *  Created on: Jan 15, 2018
 *      Author: chris
 */

#ifndef SRC_SUBSYSTEMS_ELEVATOR_H_
#define SRC_SUBSYSTEMS_ELEVATOR_H_

#include <vector>
#include "Utilities/CustomSubsystem.h"
#include "Utilities/Controllers.h"
#include "ctre/Phoenix.h"

using namespace std;

class Elevator : CustomSubsystem {
private:
	vector<CustomSubsystem*>* customSubsystemVector;

	TalonSRX* liftMotor;
	VictorSPX* liftMotorSlave;

public:
	Elevator();
	~Elevator() {}

	void init() override;
	void start() override;
	void subsystemHome() override;
	void stop() override;

	bool isElevatorFaulted();
};

#endif /* SRC_SUBSYSTEMS_ELEVATOR_H_ */

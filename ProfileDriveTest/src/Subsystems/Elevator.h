/*
 * Elevator.h
 *
 *  Created on: Jan 15, 2018
 *      Author: chris
 */

#ifndef SRC_SUBSYSTEMS_ELEVATOR_H_
#define SRC_SUBSYSTEMS_ELEVATOR_H_

#include <vector>
#include <sstream>
#include "Utilities/CustomSubsystem.h"
#include "Utilities/Controllers.h"
#include "ctre/Phoenix.h"

using namespace std;

class Elevator : CustomSubsystem {
private:
	TalonSRX* liftMotor;
	VictorSPX* liftMotorSlave;

public:
	Elevator(Controllers* robotControllers, vector<CustomSubsystem*>* subsystemVector);
	~Elevator() {}

	void init() override;
	void start() override;
	void subsystemHome() override;
	void stop() override;

	bool isElevatorFaulted();
	double getElevatorPos();
};

#endif /* SRC_SUBSYSTEMS_ELEVATOR_H_ */

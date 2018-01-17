/*
 * Elevator.cpp
 *
 *  Created on: Jan 15, 2018
 *      Author: chris
 */

#include "Subsystems/Elevator.h"

Elevator::Elevator(Controllers* robotControllers, vector<CustomSubsystem*>* subsystemVector) {
	subsystemVector->push_back(this);

	this->liftMotor = robotControllers->getLiftMotor();
	this->liftMotorSlave = robotControllers->getLiftMotorSlave();
}

bool Elevator::isElevatorFaulted() {
	return true;
}

double Elevator::getElevatorPos() {

}

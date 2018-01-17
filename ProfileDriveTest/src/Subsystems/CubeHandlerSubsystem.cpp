/*
 * Elevator.cpp
 *
 *  Created on: Jan 15, 2018
 *      Author: chris
 */

#include <Subsystems/CubeHandlerSubsystem.h>

CubeHandlerSubsystem *CubeHandlerSubsystem::instance = NULL;
CubeHandlerSubsystem::CubeHandlerSubsystem() {
	ds = &DriverStation::GetInstance();
	Controllers *robotControllers = Controllers::getInstance();

	liftMotor = robotControllers->getLiftMotor();
	liftMotorSlave = robotControllers->getLiftMotorSlave();

	requestedElevatorPos = 0;
}

CubeHandlerSubsystem* CubeHandlerSubsystem::getInstance() {
	if(instance == NULL)
		instance = new CubeHandlerSubsystem();

	return instance;
}

CubeHandlerSubsystem* CubeHandlerSubsystem::getInstance(vector<CustomSubsystem *> *subsystemVector) {
	subsystemVector->push_back(getInstance());
	return instance;
}

void CubeHandlerSubsystem::init() {

	this_thread::sleep_for(chrono::milliseconds(20));
}

void CubeHandlerSubsystem::start() {
	//runThread = true;
	//leftDriveThread = thread(&DriveBaseSubsystem::runLeftDrive, this);
}

void CubeHandlerSubsystem::subsystemHome() {

	this_thread::sleep_for(chrono::milliseconds(25));
}

void CubeHandlerSubsystem::stop() {
	/*
	cout << "drive stop called" << endl;
	runThread = false;
	if (leftDriveThread.joinable())
		leftDriveThread.join();
*/
}

double CubeHandlerSubsystem::getElevatorPos() {
	return liftMotor->GetSelectedSensorPosition(0);
}

bool CubeHandlerSubsystem::isElevatorFaulted() {
	return true;
}

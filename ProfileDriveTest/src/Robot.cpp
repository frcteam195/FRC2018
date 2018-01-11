/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

void Robot::RobotInit() {
	robotControllers = new Controllers();
	robotDrive = new DriveBaseSubsystem(robotControllers, &subsystemVector);
	hidControllerSubsystem = new HIDControllerSubsystem(robotControllers, &subsystemVector);

	for(unsigned int i = 0; i < subsystemVector.size(); i++)
		subsystemVector.at(i)->init();

	for (unsigned int i = 0; i < subsystemVector.size(); i++)
		subsystemVector.at(i)->start();
}


void Robot::Autonomous() {
	while(!IsOperatorControl() && IsEnabled()) {this_thread::sleep_for(chrono::milliseconds(100));}
}


void Robot::OperatorControl() {
	while(IsOperatorControl() && IsEnabled()) {this_thread::sleep_for(chrono::milliseconds(100));}
}

void Robot::Test() {}

START_ROBOT_CLASS(Robot)

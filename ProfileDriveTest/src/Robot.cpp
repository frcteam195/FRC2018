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

	ckAuto = new CKAutoBuilder<TalonSRX>(robotControllers->getLeftDrive1(), robotControllers->getRightDrive1(), this);
}


void Robot::Autonomous() {
	ckAuto->addAutoStep(0, 0, 2000);	//Delay for two seconds
	ckAuto->addAutoStep(1, 0.25, 1000);	//Drive forward while turning right for one second
	ckAuto->addAutoStep(0.8, -0.5, 700);	//Drive forward while turning left for 600 milliseconds
	ckAuto->addAutoStep(0.75, 0, 500);	//Drive forward for half a second
	ckAuto->addAutoStep(0, 0, 200);	//Stop
	ckAuto->start();
	while(!IsOperatorControl() && IsEnabled()) {this_thread::sleep_for(chrono::milliseconds(100));}
}


void Robot::OperatorControl() {
	while(IsOperatorControl() && IsEnabled()) {this_thread::sleep_for(chrono::milliseconds(100));}
}

void Robot::Test() {}

START_ROBOT_CLASS(Robot)

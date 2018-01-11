/*
 * Robot.h
 *
 *  Created on: Jan 10, 2018
 *      Author: roberthilton
 */

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include "WPILib.h"
#include "AHRS.h"
#include "Utilities/Controllers.h"
#include "Utilities/CustomSubsystem.h"
#include "Subsystems/DriveBaseSubsystem.h"
#include "Subsystems/HIDControllerSubsystem.h"
#include <vector>

class Robot: public SampleRobot {
public:
	void RobotInit() override;
	void Autonomous() override;
	void OperatorControl() override;
	void Test() override;
private:
	Controllers *robotControllers;
	vector<CustomSubsystem*> subsystemVector;
	DriveBaseSubsystem *robotDrive;
	HIDControllerSubsystem *hidControllerSubsystem;

	//HIDControllerSubsystem *hid;

};


#endif /* SRC_ROBOT_H_ */

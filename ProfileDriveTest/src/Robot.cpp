/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "WPILib.h"
#include <SampleRobot.h>

class Robot : public frc::SampleRobot {
public:
	Robot() {

	}

	void RobotInit() {

	}


	void Autonomous() {

	}


	void OperatorControl() override {
		while (IsOperatorControl() && IsEnabled()) {

		}
	}

	void Test() override {}

private:

};

START_ROBOT_CLASS(Robot)

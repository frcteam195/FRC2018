#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include "WPILib.h"
#include <thread>
#include <iostream>
#include "Utilities/Controllers.h"
#include "Subsystems/Intake.h"
#include "Subsystems/Shooter.h"

class Robot: public SampleRobot {
private:
	Controllers *robotControllers;
	Intake *ballIntake;
	Shooter *ballShooter;

	bool initComplete;
public:
	void RobotInit() override;
	void OperatorControl() override;
};

#endif /* SRC_ROBOT_H */

#include "Robot.h"

void Robot::RobotInit() {
	robotControllers = new Controllers();
	ballIntake = new Intake(robotControllers);
	ballShooter = new Shooter(robotControllers);

	initComplete = false;
}

void Robot::OperatorControl() {
	if(IsEnabled()&&IsOperatorControl())
	{
		//First run code
		if (!initComplete)
		{
			if(!ballShooter->isHomed()) {
				ballShooter->home();
			}
			initComplete = true;
		}

		ballIntake->start();
		ballShooter->start();
	}
	while (IsEnabled()&&IsOperatorControl()) {
		//std::cout << robotControllers->getWinchSwitch()->Get() << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

START_ROBOT_CLASS(Robot)


#include <iostream>
#include "Intake.h"
using namespace std;

Intake::Intake(Controllers *robotControllers) {
	ds = &DriverStation::GetInstance();
	armJoystick = robotControllers->getArmJoystick();
	intakeMotor = robotControllers->getIntakeMotor();
	intakeThread = std::thread();
}

void Intake::start() {
	intakeThread = std::thread(&Intake::run, this);
}

void Intake::run() {
	while(ds->IsEnabled() && ds->IsOperatorControl()) {
		if(armJoystick->GetRawButton(2)) {
			intakeMotor->Set(0.75);
		}
		else if(armJoystick->GetRawButton(3)) {
			intakeMotor->Set(-0.75);
		}
		else {
			intakeMotor->Set(0);
		}
	}
}

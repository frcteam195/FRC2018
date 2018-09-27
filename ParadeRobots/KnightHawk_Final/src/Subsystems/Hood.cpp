#include "Hood.h"

Hood::Hood(Controllers *robotControllers) {
	armJoystick = robotControllers->getArmJoystick();
	hoodMotor = robotControllers->getHoodMotor();

	homed = false;
	setpoint = 0;
	currentSetpoint = setpoint;
}

void Hood::home() {
	homed = false;
	hoodMotor->SetEncPosition(0);
	hoodMotor->SetControlMode(CANTalon::kPosition);
	hoodMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
	hoodMotor->SetClosedLoopOutputDirection(true);
	hoodMotor->SetP(5);
	hoodMotor->SetI(0);
	hoodMotor->SetD(1);
	hoodMotor->SetAllowableClosedLoopErr(15);
	hoodMotor->SetEncPosition(0);
	hoodMotor->Enable();
	hoodMotor->Set(0);
	homed = true;
}

bool Hood::isHomed() {
	return homed;
}

void Hood::raise() {
	setpoint = 700;
	hoodMotor->Set(700);
}

void Hood::lower() {
	setpoint = 0;
	hoodMotor->Set(0);
}

bool Hood::offPoint() {
	return abs(hoodMotor->GetEncPosition() - setpoint) > 100;
}

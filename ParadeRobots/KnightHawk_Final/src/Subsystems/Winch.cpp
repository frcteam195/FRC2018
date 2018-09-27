#include "Winch.h"

Winch::Winch(Controllers *robotControllers) {
	winchMotor = robotControllers->getWinchMotor();
	winchSwitch = robotControllers->getWinchSwitch();
	triggerSwitch = robotControllers->getTriggerSwitch();

	homed = false;
	loaded = false;
}

void Winch::home() {
	homed = false;
	loaded = false;

	if(winchSwitch->Get()) {
		winchMotor->Set(-0.9);
		while(winchSwitch->Get());
		winchMotor->Set(0);
	}
	winchMotor->SetEncPosition(0);
	winchMotor->SetControlMode(CANTalon::kPosition);
	winchMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
	winchMotor->SetClosedLoopOutputDirection(false);
	winchMotor->SetP(7);
	winchMotor->SetI(0);
	winchMotor->SetD(1);
	winchMotor->SetAllowableClosedLoopErr(0);
	winchMotor->SetEncPosition(0);
	winchMotor->Enable();
	winchMotor->Set(1170);
	homed = true;
	loaded = true;
}

bool Winch::isHomed() {
	return homed;
}

bool Winch::isLoaded() {
	return loaded;
}

void Winch::reload() {
	loaded = false;

	winchMotor->SetControlMode(CANTalon::kPercentVbus);
	winchMotor->Set(-0.9);
	while(winchSwitch->Get());
	winchMotor->Set(0);
	winchMotor->SetEncPosition(0);
	winchMotor->SetControlMode(CANTalon::kPosition);
	winchMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
	winchMotor->SetClosedLoopOutputDirection(false);
	winchMotor->SetP(7);
	winchMotor->SetI(0);
	winchMotor->SetD(1);
	winchMotor->SetAllowableClosedLoopErr(0);
	winchMotor->SetEncPosition(0);
	winchMotor->Enable();
	winchMotor->Set(1170);
	loaded = true;
}

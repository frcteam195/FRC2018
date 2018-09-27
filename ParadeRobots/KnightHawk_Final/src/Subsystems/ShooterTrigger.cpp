#include "ShooterTrigger.h"

ShooterTrigger::ShooterTrigger(Controllers *robotControllers) {
	triggerMotor = robotControllers->getTriggerMotor();
	triggerSwitch = robotControllers->getTriggerSwitch();

	homed = false;
}

void ShooterTrigger::home() {
	homed = false;
	triggerMotor->SetEncPosition(0);
	triggerMotor->SetControlMode(CANTalon::kPosition);
	triggerMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
	triggerMotor->SetClosedLoopOutputDirection(true);
	triggerMotor->SetP(5);
	triggerMotor->SetI(0);
	triggerMotor->SetD(1);
	triggerMotor->SetAllowableClosedLoopErr(0);
	triggerMotor->SetEncPosition(0);
	triggerMotor->Enable();
	triggerMotor->Set(0);
	homed = true;
}

bool ShooterTrigger::isHomed() {
	return homed;
}

void ShooterTrigger::shoot() {
		triggerMotor->Set(-900);
		while(triggerMotor->GetClosedLoopError() > 20);
		Wait(.75);
		triggerMotor->Set(0);
}

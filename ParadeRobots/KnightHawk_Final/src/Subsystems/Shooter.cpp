#include "Shooter.h"

Shooter::Shooter(Controllers *robotControllers) {
	ds = &DriverStation::GetInstance();
	armJoystick = robotControllers->getArmJoystick();
	intakeHood = new Hood(robotControllers);
	trigger = new ShooterTrigger(robotControllers);
	shooterWinch = new Winch(robotControllers);
	triggerSwitch = robotControllers->getTriggerSwitch();
	winchSwitch = robotControllers->getWinchSwitch();
	shooterThread = std::thread();
	homed = false;
}

void Shooter::home() {
	if(!intakeHood->isHomed()) {
		intakeHood->home();
	}
	if(!trigger->isHomed()) {
		trigger->home();
	}
	if(!shooterWinch->isHomed()) {
		shooterWinch->home();
	}
}

bool Shooter::isHomed() {
	return trigger->isHomed() && shooterWinch->isHomed();
}

void Shooter::shoot() {
	if(isHomed()) {
		if(armJoystick->GetRawButton(1) && shooterWinch->isLoaded() && !triggerSwitch->Get()) {
			intakeHood->raise();
			while(intakeHood->offPoint());
			trigger->shoot();
			std::this_thread::sleep_for(std::chrono::milliseconds(150));
			intakeHood->lower();
			reload();
		}
		else if(armJoystick->GetRawButton(1) && armJoystick->GetRawButton(2) && shooterWinch->isLoaded() && !triggerSwitch->Get()) {
			trigger->shoot();
			Wait(0.5);
			reload();
		}
	}
}

void Shooter::reload() {
	shooterWinch->reload();
}

void Shooter::start() {
	shooterThread = std::thread(&Shooter::run, this);
}

void Shooter::run() {
	while(ds->IsEnabled() && ds->IsOperatorControl()) {
		shoot();
	}
}

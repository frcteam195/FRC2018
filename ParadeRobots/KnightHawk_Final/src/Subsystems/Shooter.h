#ifndef SRC_SUBSYSTEMS_SHOOTER_H
#define SRC_SUBSYSTEMS_SHOOTER_H

#include "WPILib.h"
#include <thread>
#include "Subsystems/Subsystem.h"
#include "Utilities/Controllers.h"
#include "Subsystems/Hood.h"
#include "Subsystems/ShooterTrigger.h"
#include "Subsystems/Winch.h"

using namespace frc;

class Shooter: public CustomSubsystem {
private:
	DriverStation *ds;
	Joystick *armJoystick;
	Hood *intakeHood;
	ShooterTrigger *trigger;
	Winch *shooterWinch;
	DigitalInput *triggerSwitch;
	DigitalInput *winchSwitch;
	std::thread shooterThread;

	bool homed;
public:
	Shooter(Controllers *robotControllers);
	void home();
	bool isHomed();
	void shoot();
	void reload();
	void start();
	void run();
};

#endif /* SRC_SUBSYSTEMS_SHOOTER_H */

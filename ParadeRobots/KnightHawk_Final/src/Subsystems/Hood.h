#ifndef SRC_SUBSYSTEMS_HOOD_H
#define SRC_SUBSYSTEMS_HOOD_H

#include <Utilities/Controllers.h>
#include <iostream>

#include "WPILib.h"

class Hood {
private:
	Joystick *armJoystick;
	CANTalon *hoodMotor;

	bool homed;
	int setpoint;
	int currentSetpoint;
public:
	Hood(Controllers *robotControllers);
	void home();
	bool isHomed();
	void raise();
	void lower();
	bool offPoint();
};

#endif /* SRC_SUBSYSTEMS_HOOD_H */

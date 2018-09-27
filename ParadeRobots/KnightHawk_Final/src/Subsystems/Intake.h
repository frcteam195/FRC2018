#ifndef SRC_SUBSYSTEMS_INTAKE_H
#define SRC_SUBSYSTEMS_INTAKE_H

#include "WPILib.h"
#include <thread>
#include "Utilities/Controllers.h"

using namespace frc;

class Intake {
private:
	DriverStation *ds;
	Joystick *armJoystick;
	CANTalon *intakeMotor;
	std::thread intakeThread;
public:
	Intake(Controllers *robotControllers);
	void start();
	void run();
};

#endif /* SRC_SUBSYSTEMS_INTAKE_H */

#ifndef SRC_SUBSYSTEMS_TRIGGER_H
#define SRC_SUBSYSTEMS_TRIGGER_H

#include <Utilities/Controllers.h>
#include "WPILib.h"

class ShooterTrigger {
private:
	CANTalon *triggerMotor;
	DigitalInput *triggerSwitch;

	bool homed;
public:
	ShooterTrigger(Controllers *robotControllers);
	void home();
	bool isHomed();
	void shoot();
};

#endif /* SRC_SUBSYSTEMS_TRIGGER_H */

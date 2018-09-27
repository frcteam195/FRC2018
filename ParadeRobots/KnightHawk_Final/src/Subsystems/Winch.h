#ifndef SRC_SUBSYSTEMS_WINCH_H
#define SRC_SUBSYSTEMS_WINCH_H

#include <Utilities/Controllers.h>
#include "WPILib.h"

class Winch {
private:
	CANTalon *winchMotor;
	DigitalInput *winchSwitch;
	DigitalInput *triggerSwitch;

	bool homed;
	bool loaded;
public:
	Winch(Controllers *robotControllers);
	void home();
	bool isHomed();
	bool isLoaded();
	void reload();
};

#endif /* SRC_SUBSYSTEMS_WINCH_H */

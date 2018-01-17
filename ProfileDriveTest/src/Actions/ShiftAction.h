/*
 * ShiftAction.h
 *
 *  Created on: Mar 11, 2017
 *      Author: chris
 */

#ifndef SRC_ACTIONS_SHIFTACTION_H_
#define SRC_ACTIONS_SHIFTACTION_H_


#include <Utilities/CustomAction.h>
#include "Subsystems/DriveBaseSubsystem.h"

class ShiftAction: public CustomAction {
public:
	ShiftAction():CustomAction() {
		driveBaseSubsystem = DriveBaseSubsystem::getInstance();
	};
	~ShiftAction() {};

	void start() override {};
	void start(bool highGear) {
		driveBaseSubsystem->setGear(highGear);
	};
	void stop() override {};

protected:
	void run() override {};

private:
	DriveBaseSubsystem *driveBaseSubsystem;
};



#endif /* SRC_ACTIONS_SHIFTACTION_H_ */

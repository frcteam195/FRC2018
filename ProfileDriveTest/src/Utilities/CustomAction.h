/*
 * CustomAction.h
 *
 *  Created on: Mar 9, 2017
 *      Author: roberthilton
 */

#ifndef SRC_UTILITIES_CUSTOMACTION_H_
#define SRC_UTILITIES_CUSTOMACTION_H_

#include "Utilities/Controllers.h"
#include "Utilities/CustomSubsystem.h"
#include <mutex>
#include <vector>

using namespace std;

class CustomAction {
public:
	CustomAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector) {
		this->robotControllers = robotControllers;
		this->subsystemVector = subsystemVector;
		running = false;

		timeoutStart = 0;
		timeoutEnd = 0;
		timeoutElapsedTimeMS = 0;
	}
	virtual ~CustomAction() {};

	virtual bool isRunning() {
		return running;
	}

	virtual void start() = 0;
	virtual void stop() = 0;

protected:
	Controllers *robotControllers;
	vector<CustomSubsystem*> *subsystemVector;
	double timeoutStart, timeoutEnd;
	int timeoutElapsedTimeMS;
	bool running;

	virtual void run() = 0;
	thread runAction;

	mutex _actionMutex;
};



#endif /* SRC_UTILITIES_CUSTOMACTION_H_ */

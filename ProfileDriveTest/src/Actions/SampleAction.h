/*
 * SampleAction.h
 *
 *  Created on: Jan 10, 2018
 *      Author: roberthilton
 */

#ifndef SRC_ACTIONS_SAMPLEACTION_H_
#define SRC_ACTIONS_SAMPLEACTION_H_

#include <Utilities/CustomAction.h>
#include "Subsystems/DriveBaseSubsystem.h"

#define SAMPLE_TIMEOUT_MS 100

class SampleAction: public CustomAction {
public:
	SampleAction(Controllers *robotControllers, vector<CustomSubsystem*> *subsystemVector): CustomAction(robotControllers, subsystemVector) {
		for (unsigned int i = 0; i < subsystemVector->size(); i++) {
			if (dynamic_cast<DriveBaseSubsystem*>(subsystemVector->at(i)) != NULL)
				driveBaseSubsystem = dynamic_cast<DriveBaseSubsystem*>(subsystemVector->at(i));
		}

		avgPosTmp = 0;
	};

	~SampleAction() {}

	void start() override {
		if (_actionMutex.try_lock()) {
			running = true;
			runAction = thread(&SampleAction::run, this);
		}
	};

	void stop() override {};

protected:
	void run() override {
		driveBaseSubsystem->setDriveSpeed(0, 0);
		driveBaseSubsystem->setPosition(0);
		this_thread::sleep_for(chrono::milliseconds(40));

		driveBaseSubsystem->setDriveSpeed(0.25, -0.25);

		avgPosTmp = 0;
		timeoutStart = Timer::GetFPGATimestamp();
		do {
			avgPosTmp = driveBaseSubsystem->getAveragePosition();
			this_thread::sleep_for(chrono::milliseconds(5));
			timeoutEnd = Timer::GetFPGATimestamp();
			timeoutElapsedTimeMS = (int)((timeoutEnd - timeoutStart) * 1000);

		} while (avgPosTmp < 0.25 && timeoutElapsedTimeMS < SAMPLE_TIMEOUT_MS);

		driveBaseSubsystem->setDriveSpeed(0, 0);


		running = false;
		runAction.detach();
		_actionMutex.unlock();
	};

private:
	DriveBaseSubsystem *driveBaseSubsystem;
	double avgPosTmp;

};

#endif /* SRC_ACTIONS_SAMPLEACTION_H_ */

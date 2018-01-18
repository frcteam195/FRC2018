/*
 * HIDControllerSubsystem.h
 *
 *  Created on: Mar 9, 2017
 *      Author: roberthilton
 */

#ifndef SRC_SUBSYSTEMS_HIDCONTROLLERSUBSYSTEM_H_
#define SRC_SUBSYSTEMS_HIDCONTROLLERSUBSYSTEM_H_

#include "WPILib.h"
#include "Utilities/CustomSubsystem.h"
#include "Utilities/Controllers.h"
#include "Utilities/DriveHelper.h"
#include "Utilities/GlobalDefines.h"
#include "Utilities/KnightJoystick.h"
#include "Actions/ActionIncludes.h"
#include <thread>

#define MIN_HID_THREAD_LOOP_TIME_MS 45

using namespace frc;
using namespace std;

class HIDControllerSubsystem: public CustomSubsystem {
public:


	void init() override;
	void start() override;
	void subsystemHome() override;
	void stop() override;

	static HIDControllerSubsystem *getInstance();
	static HIDControllerSubsystem *getInstance(vector<CustomSubsystem *> *subsystemVector);

private:
	HIDControllerSubsystem();
	~HIDControllerSubsystem() {}

	static HIDControllerSubsystem *instance;

	ShiftAction *shiftAction;

	DriveBaseSubsystem *driveBaseSubsystem;

	DriveHelper *driveHelper;

	DriverStation *ds;

	KnightJoystick *driveJoystick;

	bool runThread;

	bool comingFromAuto;

	double x, y, absLeft, absRight,normalLeft,normalRight, left, right;

	thread driveJoystickThread;
	double driveJoystickThreadControlStart, driveJoystickThreadControlEnd;
	int driveJoystickThreadControlElapsedTimeMS;

	void runDriveJoystickThread();

	double sgn(double x) {
		return (x > 0) - (x < 0);
	};

};

#endif /* SRC_SUBSYSTEMS_HIDCONTROLLERSUBSYSTEM_H_ */

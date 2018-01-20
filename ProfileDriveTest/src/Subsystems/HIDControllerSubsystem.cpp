/*
 * HIDControllerSubsystem.cpp
 *
 *  Created on: Mar 9, 2017
 *      Author: roberthilton
 */

#include <Subsystems/HIDControllerSubsystem.h>

using namespace frc;
using namespace std;

HIDControllerSubsystem *HIDControllerSubsystem::instance = NULL;
HIDControllerSubsystem::HIDControllerSubsystem() {

	ds = &DriverStation::GetInstance();

	Controllers *robotControllers = Controllers::getInstance();
	driveJoystick = robotControllers->getDriveJoystick();

	shiftAction = new ShiftAction();

	driveBaseSubsystem = DriveBaseSubsystem::getInstance();

	driveHelper = new DriveHelper();

	runThread = false;
	comingFromAuto = true;

	x = 0, y = 0, absLeft = 0, absRight = 0,normalLeft = 0,normalRight = 0, left = 0, right = 0;

	driveJoystickThreadControlStart = 0;
	driveJoystickThreadControlEnd = 0;
	driveJoystickThreadControlElapsedTimeMS = 0;

}

HIDControllerSubsystem* HIDControllerSubsystem::getInstance() {
	if(instance == NULL)
		instance = new HIDControllerSubsystem();

	return instance;
}

HIDControllerSubsystem* HIDControllerSubsystem::getInstance(vector<CustomSubsystem *> *subsystemVector) {
	subsystemVector->push_back(getInstance());
	return instance;
}


void HIDControllerSubsystem::init() {

}

void HIDControllerSubsystem::start() {
	runThread = true;
	driveJoystickThread = thread(&HIDControllerSubsystem::runDriveJoystickThread, this);
}

void HIDControllerSubsystem::subsystemHome() {

}

void HIDControllerSubsystem::stop() {
	runThread = false;
	if(driveJoystickThread.joinable())
		driveJoystickThread.join();
}

void HIDControllerSubsystem::runDriveJoystickThread() {
	while (!ds->IsEnabled()) {this_thread::sleep_for(chrono::milliseconds(20));}
	subsystemHome();

	while (ds->IsAutonomous()) {
		this_thread::sleep_for(chrono::milliseconds(100));
	}

	while(runThread) {
		driveJoystickThreadControlStart = Timer::GetFPGATimestamp();

		if (driveJoystick->GetRisingEdgeButton(DRIVE_SHIFT_LOW)) {
			shiftAction->start(false);
		} else if (driveJoystick->GetRisingEdgeButton(DRIVE_SHIFT_HIGH)) {
			shiftAction->start(true);
		}

		x = driveJoystick->GetRawAxis(DRIVE_X_AXIS);
		y = -driveJoystick->GetRawAxis(DRIVE_Y_AXIS);

		if (!driveBaseSubsystem->isAutoUpdate()) {
			driveBaseSubsystem->setDriveSpeed(driveHelper->calculateOutput(y, x, driveJoystick->GetRawButton(DRIVE_IMM_TURN), driveBaseSubsystem->isHighGear()));
			//driveBaseSubsystem->setDriveSpeed((y + x) * 2300, (y - x) * 2300);
		}

		do {
			driveJoystickThreadControlEnd = Timer::GetFPGATimestamp();
			driveJoystickThreadControlElapsedTimeMS = (int) ((driveJoystickThreadControlEnd - driveJoystickThreadControlStart) * 1000);
			if (driveJoystickThreadControlElapsedTimeMS < MIN_HID_THREAD_LOOP_TIME_MS)
				this_thread::sleep_for(chrono::milliseconds(MIN_HID_THREAD_LOOP_TIME_MS - driveJoystickThreadControlElapsedTimeMS));
		} while(driveJoystickThreadControlElapsedTimeMS < MIN_HID_THREAD_LOOP_TIME_MS);

	}
}


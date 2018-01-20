/*
 * Robot.h
 *
 *  Created on: Jan 10, 2018
 *      Author: roberthilton
 */

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include "WPILib.h"
#include "AHRS.h"
#include "Autonomous/AutoProfileTest.h"
#include "Utilities/Controllers.h"
#include "Utilities/CustomSubsystem.h"
#include "Subsystems/DriveBaseSubsystem.h"
#include "Subsystems/CubeHandlerSubsystem.h"
#include "Subsystems/HIDControllerSubsystem.h"
#include "Utilities/CKAutoBuilder.h"
#include <vector>

#include "Utilities/Path/Translation2d.h"
#include "Utilities/Path/Rotation2d.h"
#include "Utilities/Path/RigidTransform2d.h"
#include "Utilities/Path/Arc.h"

class Robot: public SampleRobot {
public:
	void RobotInit() override;
	void Autonomous() override;
	void OperatorControl() override;
	void Test() override;
private:
	Controllers *robotControllers;
	vector<CustomSubsystem*> subsystemVector;

	AutoProfileTest *autoProfileTest;

	DriveBaseSubsystem *driveBaseSubsystem;
	CubeHandlerSubsystem *cubeHandlerSubsystem;
	HIDControllerSubsystem *hidControllerSubsystem;

	CKAutoBuilder<TalonSRX> *ckAuto;
};


#endif /* SRC_ROBOT_H_ */

/*
 * Elevator.h
 *
 *  Created on: Jan 15, 2018
 *      Author: chris
 */

#ifndef SRC_SUBSYSTEMS_CUBEHANDLERSUBSYSTEM_H_
#define SRC_SUBSYSTEMS_CUBEHANDLERSUBSYSTEM_H_

#include <vector>
#include "Utilities/CustomSubsystem.h"
#include "Utilities/Controllers.h"
#include "Utilities/GlobalDefines.h"
#include "ctre/Phoenix.h"

using namespace std;

class CubeHandlerSubsystem : CustomSubsystem {
private:
	CubeHandlerSubsystem();
	~CubeHandlerSubsystem() {}

	TalonSRX* liftMotor;
	VictorSPX* liftMotorSlave;
	TalonSRX* intakeMotor;
	VictorSPX* intakeMotorSlave;
	TalonSRX* intakeActuatorMotor;
	TalonSRX* intakeRotationMotor;

	DriverStation *ds;

	double requestedElevatorPos;

	static CubeHandlerSubsystem *instance;

public:
	static CubeHandlerSubsystem *getInstance();
	static CubeHandlerSubsystem *getInstance(vector<CustomSubsystem *> *subsystemVector);

	void init() override;
	void start() override;
	void subsystemHome() override;
	void stop() override;

	bool isElevatorFaulted();
	double getElevatorPos();
};

#endif /* SRC_SUBSYSTEMS_CUBEHANDLERSUBSYSTEM_H_ */

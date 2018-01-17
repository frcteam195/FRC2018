#ifndef SRC_UTILITIES_CONTROLLERS_H_
#define SRC_UTILITIES_CONTROLLERS_H_

#include <Utilities/CANSpeedControllerBuilder.h>
#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "AHRS.h"
#include "KnightJoystick.h"

class Controllers {
private:
	Controllers();
	~Controllers();

	KnightJoystick *driveJoystick;

	CANSpeedControllerBuilder *canSpeedControllerBuilder;

	TalonSRX *leftDrive1;
	TalonSRX *leftDrive2;
	TalonSRX *leftDrive3;
	TalonSRX *rightDrive1;
	TalonSRX *rightDrive2;
	TalonSRX *rightDrive3;
	DoubleSolenoid *shiftSol;

	TalonSRX* liftMotor;
	VictorSPX* liftMotorSlave;
	TalonSRX* intakeMotor;
	VictorSPX* intakeMotorSlave;
	TalonSRX* intakeActuatorMotor;
	TalonSRX* intakeRotationMotor;

	AHRS *navX;

	static Controllers *instance;

public:
	static Controllers *getInstance();

	KnightJoystick* getDriveJoystick();

	TalonSRX* getLeftDrive1();
	TalonSRX* getLeftDrive2();
	TalonSRX* getLeftDrive3();
	TalonSRX* getRightDrive1();
	TalonSRX* getRightDrive2();
	TalonSRX* getRightDrive3();
	DoubleSolenoid* getShiftSol();

	TalonSRX* getLiftMotor();
	VictorSPX* getLiftMotorSlave();
	TalonSRX* getIntakeMotor();
	VictorSPX* getIntakeMotorSlave();
	TalonSRX* getIntakeActuatorMotor();
	TalonSRX* getIntakeRotationMotor();

	AHRS*	getNavX();
};

#endif /* SRC_UTILITIES_CONTROLLERS_H_ */

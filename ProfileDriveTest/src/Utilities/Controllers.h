#ifndef SRC_UTILITIES_CONTROLLERS_H_
#define SRC_UTILITIES_CONTROLLERS_H_

#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "AHRS.h"
#include "Utilities/TalonSRXBuilder.h"
#include "KnightJoystick.h"

class Controllers {
private:
	KnightJoystick *driveJoystick;

	TalonSRXBuilder *talonSRXBuilder;

	TalonSRX *leftDrive1;
	TalonSRX *leftDrive2;
	TalonSRX *leftDrive3;
	TalonSRX *rightDrive1;
	TalonSRX *rightDrive2;
	TalonSRX *rightDrive3;
	DoubleSolenoid *shiftSol;

	AHRS *navX;

public:
	Controllers();
	~Controllers();

	KnightJoystick* getDriveJoystick();

	TalonSRX* getLeftDrive1();
	TalonSRX* getLeftDrive2();
	TalonSRX* getLeftDrive3();
	TalonSRX* getRightDrive1();
	TalonSRX* getRightDrive2();
	TalonSRX* getRightDrive3();
	DoubleSolenoid* getShiftSol();

	AHRS*	getNavX();
};

#endif /* SRC_UTILITIES_CONTROLLERS_H_ */

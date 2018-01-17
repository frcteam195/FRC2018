#include "Controllers.h"

using namespace frc;

Controllers* Controllers::instance = NULL;
Controllers::Controllers() {
	//Drive Joystick Setup
	driveJoystick = new KnightJoystick(0);

	canSpeedControllerBuilder = new CANSpeedControllerBuilder();

	//Left Drive Setup
	leftDrive1 = canSpeedControllerBuilder->createDefaultTalonSRX(1);
	leftDrive2 = canSpeedControllerBuilder->createPermanentSlaveTalonSRX(2, leftDrive1);
	leftDrive3 = canSpeedControllerBuilder->createPermanentSlaveTalonSRX(3, leftDrive1);

	//Right Drive Setup
	rightDrive1 = canSpeedControllerBuilder->createDefaultTalonSRX(4);
	rightDrive2 = canSpeedControllerBuilder->createPermanentSlaveTalonSRX(5, rightDrive1);
	rightDrive3 = canSpeedControllerBuilder->createPermanentSlaveTalonSRX(6, rightDrive1);

	//Shift Solenoid Setup
	shiftSol = new DoubleSolenoid(0, 1);

	//Elevator setup
	liftMotor = canSpeedControllerBuilder->createDefaultTalonSRX(7);
	liftMotorSlave = canSpeedControllerBuilder->createPermanentVictorSlaveToTalonSRX(8, liftMotor);
	intakeMotor = canSpeedControllerBuilder->createDefaultTalonSRX(9);
	intakeMotorSlave = canSpeedControllerBuilder->createPermanentVictorSlaveToTalonSRX(10, intakeMotor);
	intakeActuatorMotor = canSpeedControllerBuilder->createDefaultTalonSRX(11);
	intakeRotationMotor = canSpeedControllerBuilder->createDefaultTalonSRX(12);

    try {
        navX = new AHRS(SPI::Port::kMXP);
    } catch (std::exception& ex ) {
        std::string err_string = "Error instantiating navX MXP:  ";
        err_string += ex.what();
        DriverStation::ReportError(err_string.c_str());
    }
}

Controllers::~Controllers() {
}

Controllers *Controllers::getInstance() {
	if (instance == NULL)
		instance = new Controllers();

	return instance;
};

KnightJoystick* Controllers::getDriveJoystick() {
	return driveJoystick;
}

TalonSRX* Controllers::getLeftDrive1() {
	return leftDrive1;
}

TalonSRX* Controllers::getLeftDrive2() {
	return leftDrive2;
}

TalonSRX* Controllers::getLeftDrive3() {
	return leftDrive3;
}

TalonSRX* Controllers::getRightDrive1() {
	return rightDrive1;
}

TalonSRX* Controllers::getRightDrive2() {
	return rightDrive2;
}

TalonSRX* Controllers::getRightDrive3() {
	return rightDrive3;
}

DoubleSolenoid* Controllers::getShiftSol() {
	return shiftSol;
}

TalonSRX* Controllers::getIntakeMotor() {
	return intakeMotor;
}

VictorSPX* Controllers::getIntakeMotorSlave() {
	return intakeMotorSlave;
}

TalonSRX* Controllers::getIntakeActuatorMotor() {
	return intakeActuatorMotor;
}

TalonSRX* Controllers::getIntakeRotationMotor() {
	return intakeRotationMotor;
}

AHRS* Controllers::getNavX() {
	return navX;
}

TalonSRX* Controllers::getLiftMotor() {
	return liftMotor;
}

VictorSPX* Controllers::getLiftMotorSlave() {
	return liftMotorSlave;
}

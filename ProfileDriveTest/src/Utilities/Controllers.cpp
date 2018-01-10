#include "Controllers.h"

using namespace frc;

Controllers::Controllers() {
	//Drive Joystick Setup
	driveJoystick = new KnightJoystick(0);

	//Left Drive Setup
	leftDrive1 = new TalonSRX(1);
	leftDrive2 = new TalonSRX(2);
	leftDrive3 = new TalonSRX(3);

	//Right Drive Setup
	rightDrive1 = new TalonSRX(4);
	rightDrive2 = new TalonSRX(5);
	rightDrive3 = new TalonSRX(6);

	//Shift Solenoid Setup
	shiftSol = new DoubleSolenoid(0, 1);

    try {
        navX = new AHRS(SPI::Port::kMXP);
    } catch (std::exception& ex ) {
        std::string err_string = "Error instantiating navX MXP:  ";
        err_string += ex.what();
        DriverStation::ReportError(err_string.c_str());
    }
}

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

AHRS* Controllers::getNavX() {
	return navX;
}

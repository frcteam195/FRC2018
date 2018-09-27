#include <Utilities/Controllers.h>

Controllers::Controllers() {
	armJoystick = new Joystick(0);
	intakeMotor = new CANTalon(1);
	hoodMotor = new CANTalon(2);
	triggerMotor = new CANTalon(3);
	leftWinchMotor = new CANTalon(5);
	rightWinchMotor = new CANTalon(6);
	rightWinchMotor->SetControlMode(CANTalon::kFollower);
	rightWinchMotor->Set(5);
	triggerSwitch = new DigitalInput(0);
	winchSwitch = new DigitalInput(1);
	ballSensor = new DigitalInput(2);
}

Joystick* Controllers::getArmJoystick() {
	return armJoystick;
}

CANTalon* Controllers::getIntakeMotor() {
	return intakeMotor;
}

CANTalon* Controllers::getHoodMotor() {
	return hoodMotor;
}

CANTalon* Controllers::getTriggerMotor() {
	return triggerMotor;
}

CANTalon* Controllers::getWinchMotor() {
	return leftWinchMotor;
}

DigitalInput* Controllers::getTriggerSwitch() {
	return triggerSwitch;
}

DigitalInput* Controllers::getWinchSwitch() {
	return winchSwitch;
}

DigitalInput* Controllers::getBallSensor() {
	return ballSensor;
}

#ifndef SRC_UTILITIES_CONTROLLERS_H_
#define SRC_UTILITIES_CONTROLLERS_H_

#include "WPILib.h"
#include "CANTalon.h"

using namespace frc;

class Controllers {
private:
	Joystick *armJoystick;
	CANTalon *intakeMotor;
	CANTalon *hoodMotor;
	CANTalon *triggerMotor;
	CANTalon *leftWinchMotor;
	CANTalon *rightWinchMotor;
	DigitalInput *triggerSwitch;
	DigitalInput *winchSwitch;
	DigitalInput *ballSensor;
public:
	Controllers();
	Joystick* getArmJoystick();
	CANTalon* getIntakeMotor();
	CANTalon* getHoodMotor();
	CANTalon* getTriggerMotor();
	CANTalon* getWinchMotor();
	DigitalInput* getTriggerSwitch();
	DigitalInput* getWinchSwitch();
	DigitalInput* getBallSensor();
};

#endif /* SRC_UTILITIES_CONTROLLERS_H_ */

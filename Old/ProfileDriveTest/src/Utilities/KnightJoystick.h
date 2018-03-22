/*
 * KnightJoystick.h
 *
 *  Created on: Mar 11, 2017
 *      Author: roberthilton
 */

#ifndef SRC_UTILITIES_KNIGHTJOYSTICK_H_
#define SRC_UTILITIES_KNIGHTJOYSTICK_H_

#include "WPILib.h"
#include "Utilities/GlobalDefines.h"

using namespace frc;
using namespace std;

class KnightJoystick: public Joystick {
public:
	KnightJoystick(int port): Joystick(port) {
		for (int i = 0; i < 16; i++) {
			prevButtonVal[i] = false;
		}
	};

	bool GetRisingEdgeButton(int button) {
		try {
			bool currentButton = Joystick::GetRawButton(button);
			bool retVal = (currentButton != prevButtonVal[button-1]) && currentButton;
			prevButtonVal[button-1] = currentButton;
			return retVal;
		} catch(exception &ex) {
			return false;
		}
	}

	bool GetFallingEdgeButton(int button) {
		try {
			bool currentButton = Joystick::GetRawButton(button);
			bool retVal = (currentButton != prevButtonVal[button-1]) && !currentButton;
			prevButtonVal[button-1] = currentButton;
			return retVal;
		} catch(exception &ex) {
			return false;
		}
	}

private:
	bool prevButtonVal[16];
};


#endif /* SRC_UTILITIES_KNIGHTJOYSTICK_H_ */

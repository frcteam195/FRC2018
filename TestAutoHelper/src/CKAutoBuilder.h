/*
 * CKAutoBuilder.h
 *
 *  Created on: Jan 11, 2018
 *      Author: roberthilton
 */

#ifndef SRC_CKAUTOBUILDER_H_
#define SRC_CKAUTOBUILDER_H_

#if __has_include("ctre/Phoenix.h")
	#include "ctre/Phoenix.h"
#endif
#include "ctre/Phoenix.h"

enum SCControlMode {PWM_MotorController, CTRE_MotorController};

template<class T> class CKAutoBuilder {
public:
	CKAutoBuilder(T *leftDrive, T *rightDrive, SCControlMode speedControllerType) {
		this->leftDrive1 = leftDrive1;
		this->leftDrive2 = 0;
		this->leftDrive3 = 0;
		this->rightDrive1 = rightDrive1;
		this->rightDrive2 = 0;
		this->rightDrive3 = 0;

		numMotors = 2;
		scControl = speedControllerType;
	};
	CKAutoBuilder(T *leftDrive1, T *leftDrive2, T *rightDrive1, T *rightDrive2, SCControlMode speedControllerType) {
		this->leftDrive1 = leftDrive1;
		this->leftDrive2 = leftDrive2;
		this->leftDrive3 = 0;
		this->rightDrive1 = rightDrive1;
		this->rightDrive2 = rightDrive2;
		this->rightDrive3 = 0;

		numMotors = 4;
		scControl = speedControllerType;
	};
	CKAutoBuilder(T *leftDrive1, T *leftDrive2, T *leftDrive3, T *rightDrive1, T *rightDrive2, T *rightDrive3, SCControlMode speedControllerType) {
		this->leftDrive1 = leftDrive1;
		this->leftDrive2 = leftDrive2;
		this->leftDrive3 = leftDrive3;
		this->rightDrive1 = rightDrive1;
		this->rightDrive2 = rightDrive2;
		this->rightDrive3 = rightDrive3;

		numMotors = 6;
		scControl = speedControllerType;
	};

	~CKAutoBuilder() {};

	void start() {

	};

private:
	T *leftDrive1;
	T *leftDrive2;
	T *leftDrive3;
	T *rightDrive1;
	T *rightDrive2;
	T *rightDrive3;

	int numMotors;
	SCControlMode scControl;

	void setMotors(double leftOutput, double rightOutput) {
		switch(numMotors) {
			case 6:
				setSingleMotor(leftDrive3, leftOutput);
				setSingleMotor(rightDrive3, rightOutput);
				/* no break */
			case 4:
				setSingleMotor(leftDrive2, leftOutput);
				setSingleMotor(rightDrive2, rightOutput);
				/* no break */
			case 2:
				setSingleMotor(leftDrive1, leftOutput);
				setSingleMotor(rightDrive1, rightOutput);
				break;
			default:
				//ErrorState
				break;
		}
	};

	void setSingleMotor(void *sc, double output) {
		switch(scControl) {
			case SCControlMode::PWM_MotorController:
				if (dynamic_cast<SpeedController*>(sc) != NULL)
					dynamic_cast<SpeedController*>(sc)->Set(output);
				break;
			case SCControlMode::CTRE_MotorController:
				if (dynamic_cast<BaseMotorController*>(sc) != NULL)
					dynamic_cast<BaseMotorController*>(sc)->Set(ControlMode::PercentOutput, output);
				break;
			default:
				//ErrorState
				break;
		}
	};
};



#endif /* SRC_CKAUTOBUILDER_H_ */

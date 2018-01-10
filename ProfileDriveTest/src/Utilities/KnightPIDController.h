/*
 * KnightPIDController.h
 *
 *  Created on: Mar 2, 2017
 *      Author: roberthilton
 */

#ifndef SRC_UTILITIES_KNIGHTPIDCONTROLLER_H_
#define SRC_UTILITIES_KNIGHTPIDCONTROLLER_H_

#include "Utilities/GlobalDefines.h"
#include "PIDValues.h"
#include "WPILib.h"

#include <mutex>
#include <time.h>
#include <thread>
#include <arpa/inet.h>
#include <iostream>
#include <mutex>
#include <netdb.h>
#include <sys/socket.h>

using namespace std;

class KnightPIDController {
public:
	KnightPIDController(double kP, double kI, double kD, double dFilterR) {
		this->kP = kP;
		this->kI = kI;
		this->kD = kD;
		this->dFilterR = dFilterR;

		setpoint = 0;
		actualValue = 0;
		output = 0;

		minSetpoint = 0;
		maxSetpoint = 0;

		minNegOutput = 0;
		minPosOutput = 0;
		maxNegOutput = 1;
		maxPosOutput = 1;


		prevFilteredDeriv = 0;
		filteredDeriv = 0;

		currError = 0;
		prevError = 0;
		prevTime = 0;
		currTime = 0;
		errorSum = 0;
		propTerm = 0;
		integTerm = 0;
		derivTerm = 0;

		enableSetpointBounds = false;

		cout << "Construct KnightPID" << endl;
	};
	~KnightPIDController() {};

	double calculatePID(double actualValue) {
		return calculatePID(setpoint, actualValue);
	};

	double calculatePID(double setpoint, double actualValue) {
		if (enableSetpointBounds) {
			setpoint = setpoint > maxSetpoint ? maxSetpoint : setpoint;
			setpoint = setpoint < minSetpoint ? minSetpoint : setpoint;
		}

		currTime = Timer::GetFPGATimestamp();

		currError = actualValue - setpoint;
		propTerm = kP * currError;

		errorSum += currError;
		integTerm = kI * errorSum;

		derivTerm = kD * ((currError - prevError) / (currTime - prevTime));

		filteredDeriv = (1 - dFilterR) * prevFilteredDeriv + dFilterR * derivTerm;
		derivTerm = filteredDeriv;

		prevTime = currTime;

		//calculate control output
		output = propTerm + integTerm + derivTerm;

		//clamp output to min and max output value to prevent
		//saturation and prevent integral windup
		//integTerm = output > maxPosOutput ? maxPosOutput - propTerm - derivTerm : integTerm;
		//integTerm = output < maxNegOutput ? maxNegOutput - propTerm - derivTerm : integTerm;

		//prevent integral windup
		//errorSum = output > maxPosOutput ? integTerm / kI : errorSum;
		//errorSum = output < maxNegOutput ? integTerm / kI : errorSum;

		//generate new control output based on min and max and
		//integral windup.
		//output = propTerm + integTerm + derivTerm;

		output = output > minNegOutput && output < 0 ? minNegOutput : output;
		output = output < minPosOutput && output > 0 ? minPosOutput : output;
		return output;
	};

	double getSetpoint() {
		return setpoint;
	}

	void setSetpoint(double setpoint) {
		this->setpoint = setpoint;
	}

	void setGains(double kP, double kI, double kD, double dFilterR) {
		this->kP = kP;
		this->kI = kI;
		this->kD = kD;
		this->dFilterR = dFilterR;
	}

	void setSetpointBounds(double minSetpoint, double maxSetpoint) {
		this->minSetpoint = minSetpoint;
		this->maxSetpoint = maxSetpoint;
		enableSetpointBounds = true;
	}

	void setMotorOutputBounds(double minPosOutput, double maxPosOutput, double minNegOutput, double maxNegOutput) {
		this->minPosOutput = minPosOutput;
		this->maxPosOutput = maxPosOutput;
		this->minNegOutput = minNegOutput;
		this->maxNegOutput = maxNegOutput;
	}

private:

	double kP, kI, kD;

	double setpoint, actualValue;
	double minSetpoint, maxSetpoint;
	double minPosOutput, maxPosOutput, minNegOutput, maxNegOutput;
	double currTime, prevTime;
	double currError, errorSum, prevError;
	double propTerm, integTerm, derivTerm;
	double prevFilteredDeriv, filteredDeriv;
	double dFilterR;

	double output;

	bool enableSetpointBounds;

};


#endif /* SRC_UTILITIES_KNIGHTPIDCONTROLLER_H_ */

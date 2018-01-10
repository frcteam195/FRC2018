/*
 * PIDValues.h
 *
 *  Created on: Jan 10, 2017
 *      Author: roberthilton
 */

#ifndef SRC_UTILITIES_PIDVALUES_H_
#define SRC_UTILITIES_PIDVALUES_H_

struct PIDValues {
	double kP;
	double kI;
	double kD;
	double f;
	double setpoint;
	double cruiseVelocity;
	double acceleration;
	double dFilter;
};



#endif /* SRC_UTILITIES_PIDVALUES_H_ */

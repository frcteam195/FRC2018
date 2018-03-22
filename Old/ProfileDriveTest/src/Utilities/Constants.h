/*
 * Constants.h
 *
 *  Created on: Jan 10, 2018
 *      Author: roberthilton
 */

#ifndef SRC_UTILITIES_CONSTANTS_H_
#define SRC_UTILITIES_CONSTANTS_H_

const int kTimeoutMs = 10;
const double kMotorDeadband = 0.01;
const double kSensorUnitsPerRotation = 4096;
const double kEpsilon = 1E-9;
const double kReallyBigNumber = 1E9;

const double kSegmentCompletionTolerance = 0.1; // inches
const double kPathFollowingMaxAccel = 120.0; // inches per second^2
const double kPathFollowingMaxVel = 120.0; // inches per second
const double kPathFollowingProfileKp = 5.00;
const double kPathFollowingProfileKi = 0.03;
const double kPathFollowingProfileKv = 0.02;
const double kPathFollowingProfileKffv = 1.0;
const double kPathFollowingProfileKffa = 0.05;
const double kPathFollowingGoalPosTolerance = 0.75;
const double kPathFollowingGoalVelTolerance = 12.0;
const double kPathStopSteeringDistance = 9.0;



#endif /* SRC_UTILITIES_CONSTANTS_H_ */

/*
 * DriveHelper.h
 *
 *  Created on: Jan 10, 2018
 *      Author: roberthilton
 *      Ported from FRC Team 254 code
 */

#ifndef SRC_UTILITIES_DRIVEHELPER_H_
#define SRC_UTILITIES_DRIVEHELPER_H_

#include "WPILib.h"
#include <Utilities/DriveMotorValues.h>

using namespace std;
using namespace frc;

class DriveHelper {
private:
	const double kThrottleDeadband = 0.02;
	const double kWheelDeadband = 0.02;

	// These factor determine how fast the wheel traverses the "non linear" sine curve.
	const double kHighWheelNonLinearity = 0.65;
	const double kLowWheelNonLinearity = 0.5;

	const double kHighNegInertiaScalar = 4.0;

	const double kLowNegInertiaThreshold = 0.65;
	const double kLowNegInertiaTurnScalar = 3.5;
	const double kLowNegInertiaCloseScalar = 4.0;
	const double kLowNegInertiaFarScalar = 5.0;

	const double kHighSensitivity = 0.95;
	const double kLowSensitiity = 1.3;

	const double kQuickStopDeadband = 0.2;
	const double kQuickStopWeight = 0.1;
	const double kQuickStopScalar = 5.0;

    double mOldWheel;
    double mQuickStopAccumlator;
    double mNegInertiaAccumlator;

    double handleDeadband(double val, double deadband) {
    		return (abs(val) > abs(deadband)) ? val : 0.0;
    	};

    double limit(double v, double minVal, double maxVal) {
    		return min(maxVal, max(minVal, v));
    	};

    double limit(double v, double maxMagnitude) {
    		return limit(v, -maxMagnitude, maxMagnitude);
    };


public:
	DriveHelper() {
		 mOldWheel = 0;
		 mQuickStopAccumlator = 0;
		 mNegInertiaAccumlator = 0;
	};
	~DriveHelper() {};



	DriveMotorValues calculateOutput(double throttle, double wheel, bool isQuickTurn, bool isHighGear) {

        wheel = handleDeadband(wheel, kWheelDeadband);
        throttle = handleDeadband(throttle, kThrottleDeadband);
        isQuickTurn |= throttle == 0;

        double negInertia = wheel - mOldWheel;
        mOldWheel = wheel;

        double wheelNonLinearity;
        if (isHighGear) {
            wheelNonLinearity = kHighWheelNonLinearity;
            const double denominator = sin(M_PI / 2.0 * wheelNonLinearity);
            // Apply a sin function that's scaled to make it feel better.
            wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / denominator;
        } else {
            wheelNonLinearity = kLowWheelNonLinearity;
            const double denominator = sin(M_PI / 2.0 * wheelNonLinearity);
            // Apply a sin function that's scaled to make it feel better.
            wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / denominator;
        }

        double leftOutput, rightOutput, overPower;
        double sensitivity;

        double angularPower;
        double linearPower;

        // Negative inertia!
        double negInertiaScalar;
        if (isHighGear) {
            negInertiaScalar = kHighNegInertiaScalar;
            sensitivity = kHighSensitivity;
        } else {
            if (wheel * negInertia > 0) {
                // If we are moving away from 0.0, aka, trying to get more wheel.
                negInertiaScalar = kLowNegInertiaTurnScalar;
            } else {
                // Otherwise, we are attempting to go back to 0.0.
                if (abs(wheel) > kLowNegInertiaThreshold) {
                    negInertiaScalar = kLowNegInertiaFarScalar;
                } else {
                    negInertiaScalar = kLowNegInertiaCloseScalar;
                }
            }
            sensitivity = kLowSensitiity;
        }
        double negInertiaPower = negInertia * negInertiaScalar;
        mNegInertiaAccumlator += negInertiaPower;

        wheel = wheel + mNegInertiaAccumlator;
        if (mNegInertiaAccumlator > 1) {
            mNegInertiaAccumlator -= 1;
        } else if (mNegInertiaAccumlator < -1) {
            mNegInertiaAccumlator += 1;
        } else {
            mNegInertiaAccumlator = 0;
        }
        linearPower = throttle;

        // Quickturn!
        if (isQuickTurn) {
            if (abs(linearPower) < kQuickStopDeadband) {
                double alpha = kQuickStopWeight;
                mQuickStopAccumlator = (1 - alpha) * mQuickStopAccumlator
                        + alpha * limit(wheel, 1.0) * kQuickStopScalar;
            }
            overPower = 1.0;
            angularPower = wheel;
        } else {
            overPower = 0.0;
            angularPower = abs(throttle) * wheel * sensitivity - mQuickStopAccumlator;
            if (mQuickStopAccumlator > 1) {
                mQuickStopAccumlator -= 1;
            } else if (mQuickStopAccumlator < -1) {
                mQuickStopAccumlator += 1;
            } else {
                mQuickStopAccumlator = 0.0;
            }
        }

        rightOutput = leftOutput = linearPower;
        leftOutput += angularPower;
        rightOutput -= angularPower;

        if (leftOutput > 1.0) {
            rightOutput -= overPower * (leftOutput - 1.0);
            leftOutput = 1.0;
        } else if (rightOutput > 1.0) {
            leftOutput -= overPower * (rightOutput - 1.0);
            rightOutput = 1.0;
        } else if (leftOutput < -1.0) {
            rightOutput += overPower * (-1.0 - leftOutput);
            leftOutput = -1.0;
        } else if (rightOutput < -1.0) {
            leftOutput += overPower * (-1.0 - rightOutput);
            rightOutput = -1.0;
        }

        DriveMotorValues d;
        d.leftDrive = leftOutput;
        d.rightDrive = rightOutput;
        return d;
	};

};



#endif /* SRC_UTILITIES_DRIVEHELPER_H_ */

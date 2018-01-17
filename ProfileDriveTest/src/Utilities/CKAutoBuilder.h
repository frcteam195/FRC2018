/*
 * CKAutoBuilder.h
 *
 *  Created on: Jan 11, 2018
 *      Author: roberthilton
 */

#ifndef SRC_CKAUTOBUILDER_H_
#define SRC_CKAUTOBUILDER_H_

#include "ctre/Phoenix.h"

#include "WPILib.h"
#include <vector>

using namespace std;
using namespace frc;

class CKAutoStep {
public:
	CKAutoStep(double throttle, double steer, int holdTimeMS) {
		this->throttle = throttle;
		this->steer = steer;
		this->holdTimeMS = holdTimeMS;
	};

	double getThrottle() {
		return throttle;
	};

	double getHeading() {
		return steer;
	}

	int getHoldTimeMS() {
		return holdTimeMS;
	};

private:
	double throttle;
	double steer;
	int holdTimeMS;
};

struct DriveSignal {
	double leftDrive;
	double rightDrive;
};

class CheesyDriveHelper {
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
    CheesyDriveHelper() {
		 mOldWheel = 0;
		 mQuickStopAccumlator = 0;
		 mNegInertiaAccumlator = 0;
	};
	~CheesyDriveHelper() {};



	DriveSignal cheesyDrive(double throttle, double wheel, bool isQuickTurn, bool isHighGear) {

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

        DriveSignal d;
        d.leftDrive = leftOutput;
        d.rightDrive = rightOutput;
        return d;
	};

};

template<class T> class CKAutoBuilder {
public:
	CKAutoBuilder(T *leftDrive1, T *rightDrive1, RobotBase *robot) {
		this->leftDrive.push_back(leftDrive1);
		this->rightDrive.push_back(rightDrive1);

		this->robot = robot;
		this->cheesyDriveHelper = new CheesyDriveHelper();
		this->useCheesyDrive = true;
		this->leftReversed = false;
		this->rightReversed = false;
		timeoutStart = 0;
		timeoutEnd = 0;
		timeoutElapsedTimeMS = 0;
		runThread = false;
	};
	CKAutoBuilder(T *leftDrive1, T *leftDrive2, T *rightDrive1, T *rightDrive2, RobotBase *robot) { // @suppress("Class members should be properly initialized")
		CKAutoBuilder(leftDrive1, rightDrive1, robot);
		this->leftDrive.push_back(leftDrive2);
		this->rightDrive.push_back(rightDrive2);
	};
	CKAutoBuilder(T *leftDrive1, T *leftDrive2, T *leftDrive3, T *rightDrive1, T *rightDrive2, T *rightDrive3, RobotBase *robot) { // @suppress("Class members should be properly initialized")
		CKAutoBuilder(leftDrive1, leftDrive2, rightDrive1, rightDrive2, robot);
		this->leftDrive.push_back(leftDrive3);
		this->rightDrive.push_back(rightDrive3);
	};

	~CKAutoBuilder() {};

	void setCheesyDriveOnOff(bool enable) {
		if (!runThread) {
			threadLockMutex.lock();
			this->useCheesyDrive = enable;
			threadLockMutex.unlock();
		}
	};

	void addAutoStep(double throttle, double steer, int holdTimeMS) {
		if (!runThread) {
			threadLockMutex.lock();
			autoSteps.push_back(new CKAutoStep(throttle, steer, holdTimeMS));
			threadLockMutex.unlock();
		}
	};

	void driveForward() {
		if (!runThread) {
			threadLockMutex.lock();
			double driveSpeed = 0.5;
			double leftOutput = leftReversed ? -driveSpeed : driveSpeed;
			double rightOutput = rightReversed ? -driveSpeed : driveSpeed;
			setDriveOutput(leftOutput, rightOutput);
			threadLockMutex.unlock();
		}
	};

	void driveForwardDelay(int delayTimeMS) {
		if (!runThread) {
			threadLockMutex.lock();
			try {
				this_thread::sleep_for(chrono::milliseconds(delayTimeMS));
			} catch (exception &e) {

			}
			driveForward();
			threadLockMutex.unlock();
		}
	};

	void setLeftSideReversed(bool reversed) {
		if (!runThread) {
			threadLockMutex.lock();
			leftReversed = reversed;
			threadLockMutex.unlock();
		}
	};

	void setRightSideReversed(bool reversed) {
		if (!runThread) {
			threadLockMutex.lock();
			rightReversed = reversed;
			threadLockMutex.unlock();
		}
	};

	void start() {
		threadLockMutex.lock();
		runThread = true;
		threadLockMutex.unlock();
		driveThread = thread(&CKAutoBuilder::runDrive, this);
	}

private:

	vector<T*> leftDrive;
	vector<T*> rightDrive;

	RobotBase *robot;

	vector<CKAutoStep*> autoSteps;
	CheesyDriveHelper *cheesyDriveHelper;
	bool useCheesyDrive;
	bool leftReversed;
	bool rightReversed;

	double timeoutStart;
	double timeoutEnd;
	int timeoutElapsedTimeMS;

	thread driveThread;
	mutex threadLockMutex;
	bool runThread;

	const int CK_MIN_DRIVE_LOOP_TIME = 10;

    double limit(double v, double minVal, double maxVal) {
    		return min(maxVal, max(minVal, v));
    	};

    double limit(double v, double maxMagnitude) {
    		return limit(v, -maxMagnitude, maxMagnitude);
    };

    void setSingleMotor(T *t, double output) {
    		if (dynamic_cast<SpeedController*>(t) != NULL) {
    			dynamic_cast<SpeedController*>(t)->Set(output);
		} else if (dynamic_cast<BaseMotorController*>(t) != NULL) {
			dynamic_cast<BaseMotorController*>(t)->Set(ControlMode::PercentOutput, output);
		} else {
			cout << "Motor controller type not recognized!" << endl;
		}
    };

	void setDriveOutput(double leftOutput, double rightOutput) {
		leftOutput = limit(leftOutput, 1);
		rightOutput = limit(rightOutput, 1);
		leftOutput = leftReversed ? -leftOutput : leftOutput;
		rightOutput = rightReversed ? -rightOutput : rightOutput;

		for(unsigned int i = 0; i < leftDrive.size(); i++) {
			setSingleMotor(leftDrive.at(i), leftOutput);
		}
		for(unsigned int i = 0; i < rightDrive.size(); i++) {
			setSingleMotor(rightDrive.at(i), rightOutput);
		}
	};

	void setDriveOutput(DriveSignal signal) {
		setDriveOutput(signal.leftDrive, signal.rightDrive);
	};

	void runDrive() {
		try {
			for(unsigned int i = 0; i < autoSteps.size(); i++) {
				timeoutElapsedTimeMS = 0;
				timeoutEnd = 0;
				timeoutStart = Timer::GetFPGATimestamp();
				while (timeoutElapsedTimeMS < autoSteps.at(i)->getHoldTimeMS() && robot->IsEnabled() && robot->IsAutonomous()) {
					if (useCheesyDrive)
						setDriveOutput(cheesyDriveHelper->cheesyDrive(autoSteps.at(i)->getThrottle(), autoSteps.at(i)->getHeading(), false, false));
					else
						setDriveOutput(autoSteps.at(i)->getThrottle(), autoSteps.at(i)->getHeading());

					this_thread::sleep_for(chrono::milliseconds(CK_MIN_DRIVE_LOOP_TIME));
					timeoutEnd = Timer::GetFPGATimestamp();
					timeoutElapsedTimeMS = (int)((timeoutEnd - timeoutStart) * 1000);
				}

				if (!(robot->IsEnabled() && robot->IsAutonomous())) {
					break;
				}
			}
		} catch (exception &e) {
			cout << "Error in thread run!" << endl;
		}
		threadLockMutex.lock();
		runThread = false;
		threadLockMutex.unlock();
		return;
	}

};
#endif /* SRC_CKAUTOBUILDER_H_ */

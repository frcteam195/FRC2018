#ifndef SRC_SUBSYSTEMS_DRIVEBASESUBSYSTEM_H_
#define SRC_SUBSYSTEMS_DRIVEBASESUBSYSTEM_H_

#include "Utilities/Constants.h"
#include "Utilities/Controllers.h"
#include "Utilities/GlobalDefines.h"
#include "Utilities/CustomSubsystem.h"
#include "Utilities/TuneablePID.h"
#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "Utilities/DriveMotorValues.h"
#include "AHRS.h"
#include <thread>
#include <vector>
#include <iostream>

#define MIN_DRIVE_LOOP_TIME_STANDARD 10
#define MIN_DRIVE_LOOP_TIME_MP 3
#define MIN_SHIFT_LOOP_TIME 50

#define DRIVE_JOYSTICK_DEADBAND 0.1

using namespace std;
using namespace frc;

class DriveBaseSubsystem : public CustomSubsystem, public TuneablePID {
public:
	void init() override;
	void start() override;
	void subsystemHome() override;
	void stop() override;

	double getAveragePosition();
	double getLeftDrivePosition();
	double getRightDrivePosition();

	void setDriveSpeed(DriveMotorValues d);
	void setDriveSpeed(double leftDriveSpeed, double rightDriveSpeed);
	void setDriveSpeed(double leftDriveSpeed, double rightDriveSpeed, bool slowDown);
	void setHoldLowGear(bool holdLowGear);
	void setGear(bool highGear);
	void setPosition(double position);
	void startMPTrajectory();
	bool isHighGear();

	MotionProfileStatus getLeftMPStatus();
	MotionProfileStatus getRightMPStatus();

	void setMotionProfileTrajectory(vector<vector< double> *> *mpLeftBuffer, vector<vector< double> *> *mpRightBuffer);
	void setControlMode(ControlMode controlMode);

	void setDrivePID(double kP, double kI, double kD, double ff, int profileNum);
	bool isPositionWithinRange(double range);

	void setMotionMagicVelocityAccel(double vel, double accel);

	static DriveBaseSubsystem *getInstance();
	static DriveBaseSubsystem *getInstance(vector<CustomSubsystem *> *subsystemVector);

private:
	DriveBaseSubsystem();
	~DriveBaseSubsystem();

	static DriveBaseSubsystem *instance;

	ControlMode requestedControlMode;

	bool startMPLeft;
	bool startMPRight;

	void processMPLeft();
	void processMPRight();
	void processMP(TalonSRX *talonSRX, vector<vector< double> *> *mpBuffer);

	TrajectoryDuration GetTrajectoryDuration(int durationMs);

	double leftDriveThreadControlStart, leftDriveThreadControlEnd;
	int leftDriveThreadControlElapsedTimeMS;

	double rightDriveThreadControlStart, rightDriveThreadControlEnd;
	int rightDriveThreadControlElapsedTimeMS;

	double shiftThreadControlStart, shiftThreadControlEnd;
	int shiftThreadControlElapsedTimeMS;

	double sgn(double x);
	DriverStation *ds;

	double leftDriveSpeed;
	double rightDriveSpeed;
	bool highGear, holdLow;
	vector<vector< double> *> *mpLeftBuffer;
	vector<vector< double> *> *mpRightBuffer;

	bool requestSetLeftPosition;
	bool requestSetRightPosition;
	double position;

	MotionProfileStatus mpStatusLeft;
	MotionProfileStatus mpStatusRight;
	TalonSRX *leftDrive;
	TalonSRX *leftDriveSlave1;
	TalonSRX *leftDriveSlave2;
	TalonSRX *rightDrive;
	TalonSRX *rightDriveSlave1;
	TalonSRX *rightDriveSlave2;

	ControlMode driveControlMode;

	AHRS *navX;

	DoubleSolenoid *shiftSol;

	thread leftDriveThread;
	thread rightDriveThread;
	thread shiftThread;

	thread leftMPBufferProcess;
	thread rightMPBufferProcess;

	mutex shiftMutex;
	mutex holdLowMutex;

	bool runThread;

	void runLeftDrive();
	void runRightDrive();
	void shift();
};

#endif /* SRC_SUBSYSTEMS_DRIVEBASESUBSYSTEM_H_ */

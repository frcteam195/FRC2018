#include <Subsystems/DriveBaseSubsystem.h>

using namespace std;
using namespace frc;

DriveBaseSubsystem *DriveBaseSubsystem::instance = NULL;
DriveBaseSubsystem::DriveBaseSubsystem()
: TuneablePID("LeftDrive", Controllers::getInstance()->getLeftDrive1(), Controllers::getInstance()->getRightDrive1(), &rightDriveSpeed, 5808, true, false) {
	ds = &DriverStation::GetInstance();

	Controllers *robotControllers = Controllers::getInstance();
	leftDrive = robotControllers->getLeftDrive1();
	leftDriveSlave1 = robotControllers->getLeftDrive2();
	leftDriveSlave2 = robotControllers->getLeftDrive3();
	rightDrive = robotControllers->getRightDrive1();
	rightDriveSlave1 = robotControllers->getRightDrive2();
	rightDriveSlave2 = robotControllers->getRightDrive3();

	shiftSol = robotControllers->getShiftSol();

	navX = robotControllers->getNavX();

	shiftSol->Set(DoubleSolenoid::kReverse);
	highGear = false;

	holdLow = false;
	leftDriveSpeed = 0;
	rightDriveSpeed = 0;

	runThread = false;

	driveControlMode = ControlMode::PercentOutput;

	leftDriveThreadControlStart = 0;
	leftDriveThreadControlEnd = 0;
	leftDriveThreadControlElapsedTimeMS = 0;

	rightDriveThreadControlStart = 0;
	rightDriveThreadControlEnd = 0;
	rightDriveThreadControlElapsedTimeMS = 0;

	shiftThreadControlStart = 0;
	shiftThreadControlEnd = 0;
	shiftThreadControlElapsedTimeMS = 0;

	mpLeftBuffer = nullptr;
	mpRightBuffer = nullptr;

	requestSetLeftPosition = false;
	requestSetRightPosition = false;
	position = 0;

	requestedControlMode = ControlMode::MotionProfile;
	startMPLeft = false;
	startMPRight = false;
}

DriveBaseSubsystem::~DriveBaseSubsystem() {}

DriveBaseSubsystem* DriveBaseSubsystem::getInstance() {
	if(instance == NULL)
		instance = new DriveBaseSubsystem();

	return instance;
}

DriveBaseSubsystem* DriveBaseSubsystem::getInstance(vector<CustomSubsystem *> *subsystemVector) {
	subsystemVector->push_back(getInstance());
	return instance;
}

void DriveBaseSubsystem::init() {
	rightDrive->SetInverted(true);
	rightDriveSlave1->SetInverted(true);
	rightDriveSlave2->SetInverted(true);

	leftDrive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
	leftDrive->ConfigVelocityMeasurementPeriod(VelocityMeasPeriod::Period_10Ms, kTimeoutMs);
	leftDrive->ConfigVelocityMeasurementWindow(32, kTimeoutMs);

	rightDrive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
	rightDrive->ConfigVelocityMeasurementPeriod(VelocityMeasPeriod::Period_10Ms, kTimeoutMs);
	rightDrive->ConfigVelocityMeasurementWindow(32, kTimeoutMs);

	leftDrive->ConfigMotionProfileTrajectoryPeriod(10, kTimeoutMs);
	leftDrive->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs);
	rightDrive->ConfigMotionProfileTrajectoryPeriod(10, kTimeoutMs);
	rightDrive->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs);

/*
	leftDrive->ConfigNeutralDeadband(kMotorDeadband, kTimeoutMs);
	leftDriveSlave1->ConfigNeutralDeadband(kMotorDeadband, kTimeoutMs);
	leftDriveSlave2->ConfigNeutralDeadband(kMotorDeadband, kTimeoutMs);
	rightDrive->ConfigNeutralDeadband(kMotorDeadband, kTimeoutMs);
	rightDriveSlave1->ConfigNeutralDeadband(kMotorDeadband, kTimeoutMs);
	rightDriveSlave2->ConfigNeutralDeadband(kMotorDeadband, kTimeoutMs);

	*/

	setDrivePID(0, 0, 0, 2, 0);

	leftDrive->SetSelectedSensorPosition(0, 0, kTimeoutMs);
	rightDrive->SetSelectedSensorPosition(0, 0, kTimeoutMs);
	this_thread::sleep_for(chrono::milliseconds(20));
}

void DriveBaseSubsystem::start() {
	runThread = true;
	leftDriveThread = thread(&DriveBaseSubsystem::runLeftDrive, this);
	rightDriveThread = thread(&DriveBaseSubsystem::runRightDrive, this);
	shiftThread = thread(&DriveBaseSubsystem::shift, this);
}

void DriveBaseSubsystem::runLeftDrive() {
	while (!ds->IsEnabled()) {this_thread::sleep_for(chrono::milliseconds(20));}
	subsystemHome();

	while(runThread) {
		leftDriveThreadControlStart = Timer::GetFPGATimestamp();

		if (requestSetLeftPosition) {
			leftDrive->SetSelectedSensorPosition(position, 0, kTimeoutMs);
			_subsystemMutex.lock();
			requestSetLeftPosition = false;
			_subsystemMutex.unlock();
		}

		ControlMode ctrlMode = leftDrive->GetControlMode();

		if(ctrlMode != requestedControlMode) {
#ifdef DEBUG
			cout << "Changing Control Mode!" << endl;
#endif

			switch (requestedControlMode) {
				case ControlMode::MotionProfile:
					leftDrive->Set(ControlMode::MotionProfile, SetValueMotionProfile::Disable);
					break;
				case ControlMode::Velocity:
					leftDrive->Set(ControlMode::Velocity, 0);
					break;
				default:
					leftDrive->Set(ControlMode::PercentOutput, 0);
					break;
			}
			ctrlMode = leftDrive->GetControlMode();
		}

		switch (ctrlMode) {
			case ControlMode::MotionProfile:
				leftDrive->GetMotionProfileStatus(mpStatusLeft);

				if(mpStatusLeft.hasUnderrun){
					leftDrive->ClearMotionProfileHasUnderrun(kTimeoutMs);
					cout << "Left Drive Underrun" << endl;
				}

				leftDrive->ProcessMotionProfileBuffer();

#ifdef DEBUG
				cout << "Left Buffer count" << mpStatusLeft.topBufferCnt << endl;
				cout << "Left Bottom Buffer count" << mpStatusLeft.btmBufferCnt << endl;
				cout << "Left Point valid: " << mpStatusLeft.activePointValid << endl;
#endif

				if (mpStatusLeft.activePointValid && mpStatusLeft.isLast) {
					leftDrive->Set(ControlMode::MotionProfile, SetValueMotionProfile::Hold);
					cout << "Bad1Left" << endl;
				}
				else if (startMPLeft) {
					_subsystemMutex.lock();
					startMPLeft = false;
					_subsystemMutex.unlock();
					leftDrive->Set(ControlMode::MotionProfile, SetValueMotionProfile::Enable);
					cout << "Enabling Left Motion Profile" << endl;
				}
				else if (mpStatusLeft.topBufferCnt == 0 && mpStatusLeft.btmBufferCnt == 0) {
					//leftDrive->Set(ControlMode::MotionProfile, SetValueMotionProfile::Disable);
					//cout << "Bad2Left" << endl;
				}
#ifdef DEBUG
				cout << "MPModeL" << endl;
#endif
				break;
			case ControlMode::Velocity:
				leftDrive->Set(ControlMode::Velocity, leftDriveSpeed * kSensorUnitsPerRotation / 60);
				break;
			case ControlMode::PercentOutput:
			default:
				if (!ds->IsAutonomous())
					leftDrive->Set(ControlMode::PercentOutput, leftDriveSpeed);
				break;
		}

#ifdef DEBUG
		cout << "Left Speed: " << leftDrive->GetSelectedSensorVelocity(0) << endl;
		cout << "Left Power: " << leftDriveSpeed << endl;
		cout << "Gear: " << highGear << endl;
#endif

		int loopRate = (ctrlMode == ControlMode::MotionProfile ? MIN_DRIVE_LOOP_TIME_MP : MIN_DRIVE_LOOP_TIME_STANDARD);

		do {
			leftDriveThreadControlEnd = Timer::GetFPGATimestamp();
			leftDriveThreadControlElapsedTimeMS = (int) ((leftDriveThreadControlEnd - leftDriveThreadControlStart) * 1000);
			if (leftDriveThreadControlElapsedTimeMS < loopRate)
				this_thread::sleep_for(chrono::milliseconds(loopRate - leftDriveThreadControlElapsedTimeMS));
		} while(leftDriveThreadControlElapsedTimeMS < loopRate);
	}
}

void DriveBaseSubsystem::runRightDrive() {
	while (!ds->IsEnabled()) {this_thread::sleep_for(chrono::milliseconds(20));}
	subsystemHome();

	while(runThread) {
		rightDriveThreadControlStart = Timer::GetFPGATimestamp();

		if (requestSetRightPosition) {
			rightDrive->SetSelectedSensorPosition(position, 0, kTimeoutMs);
			_subsystemMutex.lock();
			requestSetRightPosition = false;
			_subsystemMutex.unlock();
		}

		ControlMode ctrlMode = rightDrive->GetControlMode();

		if(ctrlMode != requestedControlMode) {
			switch (requestedControlMode) {
				case ControlMode::MotionProfile:
					rightDrive->Set(ControlMode::MotionProfile, SetValueMotionProfile::Disable);
					break;
				case ControlMode::Velocity:
					rightDrive->Set(ControlMode::Velocity, 0);
					break;
				default:
					rightDrive->Set(ControlMode::PercentOutput, 0);
					break;
			}
			ctrlMode = rightDrive->GetControlMode();
		}

		switch (ctrlMode) {
			case ControlMode::MotionProfile:
				rightDrive->GetMotionProfileStatus(mpStatusRight);

				if(mpStatusRight.hasUnderrun){
					rightDrive->ClearMotionProfileHasUnderrun(kTimeoutMs);
					cout << "Right Drive Underrun" << endl;
				}

				rightDrive->ProcessMotionProfileBuffer();

#ifdef DEBUG
				cout << "Right Buffer count" << mpStatusRight.topBufferCnt << endl;
				cout << "Right Bottom Buffer count" << mpStatusRight.btmBufferCnt << endl;
				cout << "Right Point valid: " << mpStatusRight.activePointValid << endl;
#endif

				if (mpStatusRight.activePointValid && mpStatusRight.isLast) {
					rightDrive->Set(ControlMode::MotionProfile, SetValueMotionProfile::Hold);
					cout << "Bad1Right" << endl;
				}
				else if (startMPRight) {
					_subsystemMutex.lock();
					startMPRight = false;
					_subsystemMutex.unlock();
					rightDrive->Set(ControlMode::MotionProfile, SetValueMotionProfile::Enable);
					cout << "Enabling Right Motion Profile" << endl;
				}
				else if (mpStatusRight.topBufferCnt == 0 && mpStatusRight.btmBufferCnt == 0) {
					//rightDrive->Set(ControlMode::MotionProfile, SetValueMotionProfile::Disable);
					//cout << "Bad2Right" << endl;
				}
#ifdef DEBUG
				cout << "MPModeR" << endl;
#endif
				break;
			case ControlMode::Velocity:
				rightDrive->Set(ControlMode::Velocity, rightDriveSpeed * kSensorUnitsPerRotation / 60);
				break;
			case ControlMode::PercentOutput:
			default:
				if (!ds->IsAutonomous())
					rightDrive->Set(ControlMode::PercentOutput, rightDriveSpeed);
				break;
		}

#ifdef DEBUG
		cout << "Right Speed:" << rightDrive->GetSelectedSensorVelocity(0) << endl;
		cout << "Right Power: " << rightDriveSpeed << endl;
		cout << "Gear: " << highGear << endl;
#endif

		int loopRate = (ctrlMode == ControlMode::MotionProfile ? MIN_DRIVE_LOOP_TIME_MP : MIN_DRIVE_LOOP_TIME_STANDARD);

		do {
			rightDriveThreadControlEnd = Timer::GetFPGATimestamp();
			rightDriveThreadControlElapsedTimeMS = (int) ((rightDriveThreadControlEnd - rightDriveThreadControlStart) * 1000);
			if (rightDriveThreadControlElapsedTimeMS < loopRate)
				this_thread::sleep_for(chrono::milliseconds(loopRate - rightDriveThreadControlElapsedTimeMS));
		} while(rightDriveThreadControlElapsedTimeMS < loopRate);
	}
}

void DriveBaseSubsystem::shift() {
	while (!ds->IsEnabled()) {this_thread::sleep_for(chrono::milliseconds(20));}

	while(runThread) {
		shiftThreadControlStart = Timer::GetFPGATimestamp();

		if (holdLow) {
			shiftSol->Set(DoubleSolenoid::kReverse);
		} else if (highGear) {
			shiftSol->Set(DoubleSolenoid::kForward);
		} else {
			shiftSol->Set(DoubleSolenoid::kReverse);
		}

		do {
			shiftThreadControlEnd = Timer::GetFPGATimestamp();
			shiftThreadControlElapsedTimeMS = (int) ((shiftThreadControlEnd - shiftThreadControlStart) * 1000);
			if (shiftThreadControlElapsedTimeMS < MIN_SHIFT_LOOP_TIME)
				this_thread::sleep_for(chrono::milliseconds(MIN_SHIFT_LOOP_TIME - shiftThreadControlElapsedTimeMS));
		} while(shiftThreadControlElapsedTimeMS < MIN_SHIFT_LOOP_TIME);
	}
}

void DriveBaseSubsystem::setControlMode(ControlMode controlMode) {
	_subsystemMutex.lock();
	requestedControlMode = controlMode;
	_subsystemMutex.unlock();
}

void DriveBaseSubsystem::startMPTrajectory() {
	_subsystemMutex.lock();
	startMPLeft = true;
	startMPRight = true;
	_subsystemMutex.unlock();
}

void DriveBaseSubsystem::setMotionProfileTrajectory(vector<vector< double> *> *mpLeftBuffer, vector<vector< double> *> *mpRightBuffer) {
	this->mpLeftBuffer = mpLeftBuffer;
	this->mpRightBuffer = mpRightBuffer;
	leftMPBufferProcess = thread(&DriveBaseSubsystem::processMPLeft, this);
	rightMPBufferProcess = thread(&DriveBaseSubsystem::processMPRight, this);
	if (leftMPBufferProcess.joinable())
		leftMPBufferProcess.join();
	if (rightMPBufferProcess.joinable())
		rightMPBufferProcess.join();
}

void DriveBaseSubsystem::processMPLeft() {
	processMP(leftDrive, mpLeftBuffer);
}

void DriveBaseSubsystem::processMPRight() {
	processMP(rightDrive, mpRightBuffer);
}

void DriveBaseSubsystem::processMP(TalonSRX *talonSRX, vector<vector< double> *> *mpBuffer) {
	talonSRX->ClearMotionProfileTrajectories();

	for (int i = 0; i < (int) mpBuffer->size(); i++) {
		TrajectoryPoint point;
		try {
			point.position = mpBuffer->at(i)->at(0) * kSensorUnitsPerRotation;
			point.velocity = mpBuffer->at(i)->at(1) * kSensorUnitsPerRotation / 600;
			point.headingDeg = 0;
			point.timeDur = GetTrajectoryDuration((int)mpBuffer->at(i)->at(2));
			point.profileSlotSelect0 = 0;
		} catch (exception &ex) {
#ifdef DEBUG
			cout << "Error processing motion profile buffer" << endl;
#endif
		}

		/*
		if (highGear)
			point.profileSlotSelect0 = 1;
		else
			point.profileSlotSelect0 = 0;
		 */

		if (i == 0)
			point.zeroPos = true;	//Set at start
		else
			point.zeroPos = false;

		if((i + 1) == (int) mpBuffer->size())
			point.isLastPoint = true;
		else
			point.isLastPoint = false;

		talonSRX->PushMotionProfileTrajectory(point);
	}
}

TrajectoryDuration DriveBaseSubsystem::GetTrajectoryDuration(int durationMs) {
	/* lookup and return valid value */
	switch (durationMs) {
		case 0:		return TrajectoryDuration_0ms;
		case 5:		return TrajectoryDuration_5ms;
		case 10: 	return TrajectoryDuration_10ms;
		case 20: 	return TrajectoryDuration_20ms;
		case 30: 	return TrajectoryDuration_30ms;
		case 40: 	return TrajectoryDuration_40ms;
		case 50: 	return TrajectoryDuration_50ms;
		case 100: 	return TrajectoryDuration_100ms;
	}
	printf("Trajectory Duration not supported - use configMotionProfileTrajectoryPeriod instead\n");
	return TrajectoryDuration_100ms;
}

void DriveBaseSubsystem::setDrivePID(double kP, double kI, double kD, double ff, int profileNum) {
	leftDrive->Config_kP(profileNum, kP, kTimeoutMs);
	leftDrive->Config_kI(profileNum, kI, kTimeoutMs);
	leftDrive->Config_kD(profileNum, kD, kTimeoutMs);
	leftDrive->Config_kF(profileNum, ff, kTimeoutMs);
	rightDrive->Config_kP(profileNum, kP, kTimeoutMs);
	rightDrive->Config_kI(profileNum, kI, kTimeoutMs);
	rightDrive->Config_kD(profileNum, kD, kTimeoutMs);
	rightDrive->Config_kF(profileNum, ff, kTimeoutMs);
}

void DriveBaseSubsystem::setDriveSpeed(DriveMotorValues d) {
	setDriveSpeed(d.leftDrive, d.rightDrive, false);
}

void DriveBaseSubsystem::setDriveSpeed(double leftDriveSpeed, double rightDriveSpeed) {
	setDriveSpeed(leftDriveSpeed, rightDriveSpeed, false);
}

void DriveBaseSubsystem::setDriveSpeed(double leftDriveSpeed, double rightDriveSpeed, bool slowDown) {
	_subsystemMutex.lock();
	if(slowDown) {
		this->leftDriveSpeed = leftDriveSpeed / 2.2;
		this->rightDriveSpeed = rightDriveSpeed / 2.2;
	}
	else {
		this->leftDriveSpeed = leftDriveSpeed;
		this->rightDriveSpeed = rightDriveSpeed;
	}
	_subsystemMutex.unlock();
}

void DriveBaseSubsystem::setHoldLowGear(bool holdLowGear) {
	holdLowMutex.lock();
	this->holdLow = holdLowGear;
	holdLowMutex.unlock();
}

void DriveBaseSubsystem::setGear(bool highGear) {
	shiftMutex.lock();
	this->highGear = highGear;
	shiftMutex.unlock();
}

void DriveBaseSubsystem::subsystemHome() {
	navX->ZeroYaw();
	leftDrive->SetSelectedSensorPosition(0, 0, kTimeoutMs);
	rightDrive->SetSelectedSensorPosition(0, 0, kTimeoutMs);
	this_thread::sleep_for(chrono::milliseconds(25));
}

void DriveBaseSubsystem::stop() {
	cout << "drive stop called" << endl;
	runThread = false;
	if (leftDriveThread.joinable())
		leftDriveThread.join();

	if (rightDriveThread.joinable())
		rightDriveThread.join();

	if (shiftThread.joinable())
		shiftThread.join();

	shiftSol->Set(DoubleSolenoid::kReverse);
	holdLow = false;
	leftDrive->Set(ControlMode::PercentOutput, 0);
	rightDrive->Set(ControlMode::PercentOutput, 0);
}

double DriveBaseSubsystem::getAveragePosition() {
	return (abs(leftDrive->GetSelectedSensorPosition(0)) + abs(rightDrive->GetSelectedSensorPosition(0))) / 2;
}

double DriveBaseSubsystem::getLeftDrivePosition() {
	return leftDrive->GetSelectedSensorPosition(0);
}

double DriveBaseSubsystem::getRightDrivePosition() {
	return rightDrive->GetSelectedSensorPosition(0);
}

void DriveBaseSubsystem::setMotionMagicVelocityAccel(double vel, double accel) {
	leftDrive->ConfigMotionCruiseVelocity(vel, kTimeoutMs);
	leftDrive->ConfigMotionAcceleration(accel, kTimeoutMs);
	rightDrive->ConfigMotionCruiseVelocity(vel, kTimeoutMs);
	rightDrive->ConfigMotionAcceleration(accel, kTimeoutMs);
}

bool DriveBaseSubsystem::isPositionWithinRange(double range) {
	return ((abs(leftDrive->GetSelectedSensorPosition(0)) - abs(leftDriveSpeed)) < range && (abs(rightDrive->GetSelectedSensorPosition(0)) - abs(rightDriveSpeed)) < range);
}

void DriveBaseSubsystem::setPosition(double position) {
	_subsystemMutex.lock();
	requestSetLeftPosition = true;
	requestSetRightPosition = true;
	this->position = position;
	_subsystemMutex.unlock();
}

bool DriveBaseSubsystem::isHighGear() {
	return highGear;
}

double DriveBaseSubsystem::sgn(double x) {
    return (x > 0) - (x < 0);
}

MotionProfileStatus DriveBaseSubsystem::getLeftMPStatus() {
	return mpStatusLeft;
}

MotionProfileStatus DriveBaseSubsystem::getRightMPStatus() {
	return mpStatusRight;
}

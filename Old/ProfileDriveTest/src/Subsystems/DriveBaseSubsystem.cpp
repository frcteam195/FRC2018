#include <Subsystems/DriveBaseSubsystem.h>

using namespace std;
using namespace frc;

DriveBaseSubsystem *DriveBaseSubsystem::instance = NULL;
DriveBaseSubsystem::DriveBaseSubsystem()
: TuneablePID("Drive", Controllers::getInstance()->getLeftDrive1(), &leftDriveSpeed, 5808, false, false) {
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

	driveThreadControlStart = 0;
	driveThreadControlEnd = 0;
	driveThreadControlElapsedTimeMS = 0;

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
	rightDrive->SetSensorPhase(false);

	leftDrive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
	leftDrive->ConfigVelocityMeasurementPeriod(VelocityMeasPeriod::Period_10Ms, kTimeoutMs);
	leftDrive->ConfigVelocityMeasurementWindow(32, kTimeoutMs);

	rightDrive->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
	rightDrive->ConfigVelocityMeasurementPeriod(VelocityMeasPeriod::Period_10Ms, kTimeoutMs);
	rightDrive->ConfigVelocityMeasurementWindow(32, kTimeoutMs);

	leftDrive->ConfigMotionProfileTrajectoryPeriod(0, kTimeoutMs);
	leftDrive->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs);
	rightDrive->ConfigMotionProfileTrajectoryPeriod(0, kTimeoutMs);
	rightDrive->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, kTimeoutMs);

/*
	leftDrive->ConfigNeutralDeadband(kMotorDeadband, kTimeoutMs);
	leftDriveSlave1->ConfigNeutralDeadband(kMotorDeadband, kTimeoutMs);
	leftDriveSlave2->ConfigNeutralDeadband(kMotorDeadband, kTimeoutMs);
	rightDrive->ConfigNeutralDeadband(kMotorDeadband, kTimeoutMs);
	rightDriveSlave1->ConfigNeutralDeadband(kMotorDeadband, kTimeoutMs);
	rightDriveSlave2->ConfigNeutralDeadband(kMotorDeadband, kTimeoutMs);

	*/

	setLeftDrivePID(0.2, 0, 2, 0.06, 0);
	setRightDrivePID(0.1, 0, 2, 0.0635, 0);
	//setDrivePID(0, 0, 0, 0, 0);

	leftDrive->SetSelectedSensorPosition(0, 0, kTimeoutMs);
	rightDrive->SetSelectedSensorPosition(0, 0, kTimeoutMs);
	this_thread::sleep_for(chrono::milliseconds(20));
}

void DriveBaseSubsystem::start() {
	runThread = true;
	driveThread = thread(&DriveBaseSubsystem::runDrive, this);
	shiftThread = thread(&DriveBaseSubsystem::shift, this);
}

void DriveBaseSubsystem::runDrive() {
	while (!ds->IsEnabled()) {this_thread::sleep_for(chrono::milliseconds(20));}
	subsystemHome();

	while(runThread) {
		driveThreadControlStart = Timer::GetFPGATimestamp();

		if (requestSetLeftPosition) {
			leftDrive->SetSelectedSensorPosition(position, 0, kTimeoutMs);
			rightDrive->SetSelectedSensorPosition(position, 0, kTimeoutMs);
			_subsystemMutex.lock();
			requestSetLeftPosition = false;
			_subsystemMutex.unlock();
		}

		ControlMode ctrlMode = leftDrive->GetControlMode();

		if(ctrlMode != requestedControlMode) {
			cout << "Changing Control Mode!" << endl;

			switch (requestedControlMode) {
				case ControlMode::MotionProfile:
					leftDrive->Set(ControlMode::MotionProfile, SetValueMotionProfile::Disable);
					rightDrive->Set(ControlMode::MotionProfile, SetValueMotionProfile::Disable);
					break;
				case ControlMode::Velocity:
					leftDrive->Set(ControlMode::Velocity, 0);
					rightDrive->Set(ControlMode::Velocity, 0);
					break;
				default:
					leftDrive->Set(ControlMode::PercentOutput, 0);
					rightDrive->Set(ControlMode::PercentOutput, 0);
					break;
			}
			ctrlMode = leftDrive->GetControlMode();
		}

		switch (ctrlMode) {
			case ControlMode::MotionProfile:
				leftDrive->GetMotionProfileStatus(mpStatusLeft);
				rightDrive->GetMotionProfileStatus(mpStatusRight);

				if(mpStatusLeft.hasUnderrun){
					leftDrive->ClearMotionProfileHasUnderrun(kTimeoutMs);
					cout << "Left Drive Underrun" << endl;
				}

				if(mpStatusRight.hasUnderrun){
					rightDrive->ClearMotionProfileHasUnderrun(kTimeoutMs);
					cout << "Right Drive Underrun" << endl;
				}

				leftDrive->ProcessMotionProfileBuffer();
				rightDrive->ProcessMotionProfileBuffer();

#ifdef DEBUG
				cout << "Left Buffer count" << mpStatusLeft.topBufferCnt << endl;
				cout << "Left Bottom Buffer count" << mpStatusLeft.btmBufferCnt << endl;
				cout << "Left Point valid: " << mpStatusLeft.activePointValid << endl;
				cout << "Right Buffer count" << mpStatusRight.topBufferCnt << endl;
				cout << "Right Bottom Buffer count" << mpStatusRight.btmBufferCnt << endl;
				cout << "Right Point valid: " << mpStatusRight.activePointValid << endl;
#endif

				if (mpStatusLeft.activePointValid && mpStatusLeft.isLast) {
					leftDrive->Set(ControlMode::MotionProfile, SetValueMotionProfile::Hold);
					cout << "HoldLeft" << endl;
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

				if (mpStatusRight.activePointValid && mpStatusRight.isLast) {
					rightDrive->Set(ControlMode::MotionProfile, SetValueMotionProfile::Hold);
					cout << "HoldRight" << endl;
				}
				else if (startMPRight) {
					_subsystemMutex.lock();
					startMPRight = false;
					_subsystemMutex.unlock();
					rightDrive->Set(ControlMode::MotionProfile, SetValueMotionProfile::Enable);
					cout << "Enabling Left Motion Profile" << endl;
				}
				else if (mpStatusRight.topBufferCnt == 0 && mpStatusRight.btmBufferCnt == 0) {
					//rightDrive->Set(ControlMode::MotionProfile, SetValueMotionProfile::Disable);
					//cout << "Bad2Right" << endl;
				}

				break;
			case ControlMode::Velocity:
				leftDrive->Set(ControlMode::Velocity, leftDriveSpeed * kSensorUnitsPerRotation / 600);
				rightDrive->Set(ControlMode::Velocity, rightDriveSpeed * kSensorUnitsPerRotation / 600);
				break;
			case ControlMode::PercentOutput:
			default:
				leftDrive->Set(ControlMode::PercentOutput, leftDriveSpeed);
				rightDrive->Set(ControlMode::PercentOutput, rightDriveSpeed);
				break;
		}

#ifdef DEBUG
		cout << "Left Speed: " << leftDrive->GetSelectedSensorVelocity(0) << endl;
		cout << "Left Power: " << leftDriveSpeed << endl;
		cout << "Gear: " << highGear << endl;
		cout << "Right Speed:" << rightDrive->GetSelectedSensorVelocity(0) << endl;
		cout << "Right Power: " << rightDriveSpeed << endl;
		cout << "Gear: " << highGear << endl;
#endif

		int loopRate = (ctrlMode == ControlMode::MotionProfile ? MIN_DRIVE_LOOP_TIME_MP : MIN_DRIVE_LOOP_TIME_STANDARD);

		do {
			driveThreadControlEnd = Timer::GetFPGATimestamp();
			driveThreadControlElapsedTimeMS = (int) ((driveThreadControlEnd - driveThreadControlStart) * 1000);
			if (driveThreadControlElapsedTimeMS < loopRate)
				this_thread::sleep_for(chrono::milliseconds(loopRate - driveThreadControlElapsedTimeMS));
		} while(driveThreadControlElapsedTimeMS < loopRate);
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

ControlMode DriveBaseSubsystem::getControlMode() {
	return requestedControlMode;
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
			point.profileSlotSelect1 = 0;
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
	setLeftDrivePID(kP, kI, kD, ff, profileNum);
	setRightDrivePID(kP, kI, kD, ff, profileNum);
}

void DriveBaseSubsystem::setLeftDrivePID(double kP, double kI, double kD, double ff, int profileNum) {
	leftDrive->Config_kP(profileNum, kP, kTimeoutMs);
	leftDrive->Config_kI(profileNum, kI, kTimeoutMs);
	leftDrive->Config_kD(profileNum, kD, kTimeoutMs);
	leftDrive->Config_kF(profileNum, ff, kTimeoutMs);
}

void DriveBaseSubsystem::setRightDrivePID(double kP, double kI, double kD, double ff, int profileNum) {
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
	if (driveThread.joinable())
		driveThread.join();


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

#include <Subsystems/DriveBaseSubsystem.h>

using namespace std;
using namespace frc;

DriveBaseSubsystem::DriveBaseSubsystem(Controllers *robotControllers, vector<CustomSubsystem *> *subsystemVector) {
	ds = &DriverStation::GetInstance();
	subsystemVector->push_back(this);

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
}

DriveBaseSubsystem::~DriveBaseSubsystem() {}

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

		switch (leftDrive->GetControlMode()) {
			case ControlMode::MotionProfile:
				leftDrive->GetMotionProfileStatus(mpStatusLeft);

				leftDrive->ProcessMotionProfileBuffer();


				cout << "Left Buffer count" << mpStatusLeft.topBufferCnt << endl;
				cout << "Left Bottom Buffer count" << mpStatusLeft.btmBufferCnt << endl;
				cout << "Left Point valid: " << mpStatusLeft.activePointValid << endl;

				if (mpStatusLeft.btmBufferCnt > 0)
					leftDrive->Set(ControlMode::MotionProfile, SetValueMotionProfile::Enable);
				else
					leftDrive->Set(ControlMode::MotionProfile, SetValueMotionProfile::Hold);

				break;
			default:
				leftDrive->Set(ControlMode::PercentOutput, leftDriveSpeed);
				break;
		}
/*
		cout << "Left Speed: " << leftDrive->GetSpeed() << endl;
		cout << "Left Power: " << leftDriveSpeed << endl;
		cout << "Gear: " << highGear << endl;*/

		do {
			leftDriveThreadControlEnd = Timer::GetFPGATimestamp();
			leftDriveThreadControlElapsedTimeMS = (int) ((leftDriveThreadControlEnd - leftDriveThreadControlStart) * 1000);
			if (leftDriveThreadControlElapsedTimeMS < MIN_DRIVE_LOOP_TIME)
				this_thread::sleep_for(chrono::milliseconds(MIN_DRIVE_LOOP_TIME - leftDriveThreadControlElapsedTimeMS));
		} while(leftDriveThreadControlElapsedTimeMS < MIN_DRIVE_LOOP_TIME);
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

		switch (rightDrive->GetControlMode()) {
			case ControlMode::MotionProfile:
				rightDrive->GetMotionProfileStatus(mpStatusRight);

				rightDrive->ProcessMotionProfileBuffer();


				cout << "Right Buffer count" << mpStatusRight.topBufferCnt << endl;
				cout << "Right Bottom Buffer count" << mpStatusRight.btmBufferCnt << endl;
				cout << "Right Point valid: " << mpStatusRight.activePointValid << endl;

				if (mpStatusRight.btmBufferCnt > 0)
					rightDrive->Set(ControlMode::MotionProfile, SetValueMotionProfile::Enable);
				else
					rightDrive->Set(ControlMode::MotionProfile, SetValueMotionProfile::Hold);

				break;
			default:
				rightDrive->Set(ControlMode::PercentOutput, rightDriveSpeed);
				break;
		}
/*
		cout << "Right Speed:" << rightDrive->GetSpeed() << endl;
		cout << "Right Power: " << rightDriveSpeed << endl;
		cout << "Gear: " << highGear << endl;*/

		do {
			rightDriveThreadControlEnd = Timer::GetFPGATimestamp();
			rightDriveThreadControlElapsedTimeMS = (int) ((rightDriveThreadControlEnd - rightDriveThreadControlStart) * 1000);
			if (rightDriveThreadControlElapsedTimeMS < MIN_DRIVE_LOOP_TIME)
				this_thread::sleep_for(chrono::milliseconds(MIN_DRIVE_LOOP_TIME - rightDriveThreadControlElapsedTimeMS));
		} while(rightDriveThreadControlElapsedTimeMS < MIN_DRIVE_LOOP_TIME);
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

void DriveBaseSubsystem::processMotionProfile(vector<vector< double> *> *mpLeftBuffer, vector<vector< double> *> *mpRightBuffer) {
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
	TrajectoryPoint point;

	for (int i = 0; i < (int) mpBuffer->size(); i++) {
		try {
			point.position = mpBuffer->at(i)->at(0);
			point.velocity = mpBuffer->at(i)->at(1);
			//point.timeDurMs = (int)mpBuffer->at(i)->at(2);
		} catch (exception &ex) {
			cout << "Error processing motion profile buffer" << endl;
		}
		//point.velocityOnly = false;

		if (highGear)
			point.profileSlotSelect = 1;
		else
			point.profileSlotSelect = 0;

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
	_subsystemMutex.lock();
	this->leftDriveSpeed = d.leftDrive;
	this->rightDriveSpeed = d.rightDrive;
	_subsystemMutex.unlock();
}

void DriveBaseSubsystem::setDriveSpeed(double leftDriveSpeed, double rightDriveSpeed) {
	_subsystemMutex.lock();
	this->leftDriveSpeed = leftDriveSpeed;
	this->rightDriveSpeed = rightDriveSpeed;
	_subsystemMutex.unlock();
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

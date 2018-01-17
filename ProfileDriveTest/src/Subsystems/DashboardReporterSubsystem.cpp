/*
 * DashboardReporterSubsystem.cpp
 *
 *  Created on: Apr 26, 2017
 *      Author: roberthilton
 */

#include <Subsystems/DashboardReporterSubsystem.h>

using namespace std;
using namespace frc;

DashboardReporterSubsystem *DashboardReporterSubsystem::instance = NULL;
DashboardReporterSubsystem::DashboardReporterSubsystem(int udpPort) {
	this->udpPort = udpPort;
	this->robotControllers = robotControllers;
<<<<<<< HEAD
=======
	subsystemVector->push_back(this);
	this->elevator =
	this->elevator =
>>>>>>> branch 'master' of https://guitar24t@bitbucket.org/cyberknightsprogramming/frc2018.git
	recvlenReceive = 0;
	recvlenSend = 0;
	fd = 0;
	addrlen = 0;
	status = 0;
	runThread = false;

	driveBaseSubsystem = DriveBaseSubsystem::getInstance();
	cubeHandlerSubsystem = CubeHandlerSubsystem::getInstance();

	dashboardSendThreadControlStart = 0;
	dashboardSendThreadControlEnd = 0;
	dashboardSendThreadControlElapsedTimeMS = 0;
<<<<<<< HEAD

	elevatorIsFaulted = false;
	intakeOnChris = false;
	intakeOnJake = false;
	hasCube = false;
	elevatorPos = 0;
	intakeRot = 0;
	frontCamera = true;
	manualClimb = false;
	climbLevel = 0;
=======
>>>>>>> branch 'master' of https://guitar24t@bitbucket.org/cyberknightsprogramming/frc2018.git
}

DashboardReporterSubsystem::~DashboardReporterSubsystem() {}

DashboardReporterSubsystem *DashboardReporterSubsystem::getInstance() {
	if (instance == NULL)
		instance = new DashboardReporterSubsystem(DEFAULT_DASHBOARD_UDP_PORT);

	return instance;
};

void DashboardReporterSubsystem::init() {
	//cout << "Init start for VisionReceiver!\n";
	addrlen = sizeof(remoteAddr); /* length of addresses */

	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		cout << "cannot create socket\n";
		status = 1;
		return;
	}

	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 10000;
	if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0)
	    cout << "Error setting socket timeout" << endl;

	int bufferLength = BUFSIZE;

	if (setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &bufferLength, sizeof(bufferLength)) == -1)
		cout << "Error setting socket buffer length" << endl;

	/* bind the socket to any valid IP address and a specific port */

	memset((char *) &localAddr, 0, sizeof(localAddr));
	localAddr.sin_family = AF_INET;
	localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	localAddr.sin_port = htons(udpPort);

	remoteAddr.sin_family = AF_INET;
	//remoteAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	//remoteAddr.sin_port = htons(0);
	inet_aton("10.1.95.14", &(remoteAddr.sin_addr));
	remoteAddr.sin_port = htons(udpPort);

	if (bind(fd, (struct sockaddr *) &localAddr, sizeof(localAddr)) < 0) {
		cout << "bind failed";
		status = 2;
		return;
	}
	status = 0;
	//cout << "Init finish for VisionReceiver!\n";
}

void DashboardReporterSubsystem::start() {
	if (status != 0)
		return;

	runThread = true;
	udpSendThread = thread(&DashboardReporterSubsystem::runUDPSend, this);
}

void DashboardReporterSubsystem::stop() {
	if (status != 0)
		return;

	runThread = false;
	if (udpSendThread.joinable())
		udpSendThread.join();

}

void DashboardReporterSubsystem::runUDPSend() {
	while(runThread) {
		dashboardSendThreadControlStart = Timer::GetFPGATimestamp();
		stringstream sendStr;
		sendStr << "ElevatorIsFaulted:" << elevator->isElevatorFaulted();
	}
}

void DashboardReporterSubsystem::runUDPSend() {
	while (runThread)
	{
		dashboardSendThreadControlStart = Timer::GetFPGATimestamp();
		string sendStr = "ElevatorPos:" + to_string(cubeHandlerSubsystem->getElevatorPos()) +";";
		if (cubeHandlerSubsystem->isElevatorFaulted()) {
			sendStr += "ElevatorFault:True;";
		} else {
			sendStr += "ElevatorFault:False;";
		}
		if (robotControllers->getDriveJoystick()->GetRawAxis(2) < -0.5) {
			sendStr += "IntakeOnChris:True;";
		} else {
			sendStr += "IntakeOnChris:False;";
		}
		if (robotControllers->getDriveJoystick()->GetRawButton(0)) {
			sendStr += "IntakeOnMatt:True;";
		} else {
			sendStr += "IntakeOnMatt:False;";
		}

		//_mutex.lock();

		//_mutex.unlock();

		recvlenSend = sendto(fd, sendStr.c_str(), sendStr.length()+1, 0, (sockaddr *) &remoteAddr, addrlen);
		if (recvlenSend > 0) {
			//cout << "Packet Sent!" << endl;;
		} else {
			;
		}

		//Ensure that we do not try to run faster than the sample period
		do {
			dashboardSendThreadControlEnd = Timer::GetFPGATimestamp();
			dashboardSendThreadControlElapsedTimeMS = (int) ((dashboardSendThreadControlEnd - dashboardSendThreadControlStart) * 1000);
			if (dashboardSendThreadControlElapsedTimeMS < DASHBOARD_SEND_RATE_MS)
				this_thread::sleep_for(chrono::milliseconds(DASHBOARD_SEND_RATE_MS - dashboardSendThreadControlElapsedTimeMS));
		} while(dashboardSendThreadControlElapsedTimeMS < DASHBOARD_SEND_RATE_MS);
		//cout << "loop rate: " << elapsedTimeMS << endl;
	}
}

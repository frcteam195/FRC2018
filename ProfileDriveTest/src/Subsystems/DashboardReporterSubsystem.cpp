/*
 * DashboardReporterSubsystem.cpp
 *
 *  Created on: Apr 26, 2017
 *      Author: roberthilton
 */

#include <Subsystems/DashboardReporterSubsystem.h>

using namespace std;
using namespace frc;

DashboardReporterSubsystem::DashboardReporterSubsystem(int udpPort, Controllers *robotControllers, vector<CustomSubsystem *> *subsystemVector) {
	this->udpPort = udpPort;
	this->robotControllers = robotControllers;
	subsystemVector->push_back(this);
	recvlenReceive = 0;
	recvlenSend = 0;
	fd = 0;
	addrlen = 0;
	status = 0;
	runThread = false;

	dashboardSendThreadControlStart = 0;
	dashboardSendThreadControlEnd = 0;
	dashboardSendThreadControlElapsedTimeMS = 0;

	visionEnabled = false;
	onTarget = false;
	jetsonOperational = false;
}

DashboardReporterSubsystem::~DashboardReporterSubsystem() {}

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
	while (runThread)
	{
		dashboardSendThreadControlStart = Timer::GetFPGATimestamp();
		string sendStr = "WheelSpeed:" + to_string(shooterSubsystem->getSetpoint()) +";";
		sendStr += "HoodPos:" + to_string(shooterSubsystem->getHoodSetpoint()) +";";
		sendStr += "TurretPos:" + to_string(turretSubsystem->getTurretPos()) +";";
		if (turretSubsystem->hasError()) {
			sendStr += "TurretError:True;";
		} else {
			sendStr += "TurretError:False;";
		}
		if (robotControllers->getDriveJoystick()->GetRawAxis(2) < -0.5) {
			sendStr += "IntakeOnChris:True;";
		} else {
			sendStr += "IntakeOnChris:False;";
		}
		if (robotControllers->getButtonBox1()->GetRawButton(INTAKE_IN_BUTTON) || robotControllers->getArmJoystick()->GetRawButton(ARM_INTAKE_IN_BUTTON)) {
			sendStr += "IntakeOnMatt:True;";
		} else {
			sendStr += "IntakeOnMatt:False;";
		}

		//_mutex.lock();
		if (onTarget) {
			sendStr += "OnTarget:True;";
		} else {
			sendStr += "OnTarget:False;";
		}
		if (jetsonOperational) {
			sendStr += "JetsonOperational:True;";
		} else {
			sendStr += "JetsonOperational:False;";
		}
		if (visionEnabled) {
			sendStr += "VisionEnabled:True;";
		} else {
			sendStr += "VisionEnabled:False;";
		}
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

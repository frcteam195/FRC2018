/*
 * DashboardReporterSubsystem.h
 *
 *  Created on: Apr 26, 2017
 *      Author: roberthilton
 */

#ifndef SRC_SUBSYSTEMS_DASHBOARDREPORTERSUBSYSTEM_H_
#define SRC_SUBSYSTEMS_DASHBOARDREPORTERSUBSYSTEM_H_

#define BUFSIZE 2048
#define DASHBOARD_SEND_RATE_MS 25

#include "Utilities/CustomSubsystem.h"
#include "Subsystems/Elevator.h"
#include "Subsystems/DriveBaseSubsystem.h"

#include "WPILib.h"

#include <arpa/inet.h>
#include <stdio.h>
#include <cstring>
#include <iostream>
#include <mutex>
#include <netdb.h>
#include <sys/socket.h>
#include <cstring>
#include <time.h>
#include <thread>
#include <vector>

using namespace std;

class DashboardReporterSubsystem: public CustomSubsystem {
private:
	double dashboardSendThreadControlStart, dashboardSendThreadControlEnd;
	int dashboardSendThreadControlElapsedTimeMS;

	sockaddr_in localAddr; /* our address */
	sockaddr_in remoteAddr; /* remote address */
	socklen_t addrlen;

	int recvlenReceive;
	int recvlenSend;
	int fd;
	unsigned char buf[BUFSIZE];

	int status;

	bool runThread;

	int udpPort;

	thread udpSendThread;
	mutex _mutex;

	bool elevatorIsFaulted;
	bool intakeOnChris;
	bool intakeOnJake;
	bool hasCube;
	double elevatorPos;
	double intakeRot;
	bool frontCamera;
	bool manualClimb;
	double climbLevel;

	Controllers* robotControllers;
	Elevator* elevator;

	void runUDPSend();

public:
	DashboardReporterSubsystem(int udpPort, Controllers *robotControllers, vector<CustomSubsystem *> *subsystemVector);
	~DashboardReporterSubsystem();

	void init() override;
	void start() override;
	void stop() override;
	void subsystemHome() override {};
};

#endif /* SRC_SUBSYSTEMS_DASHBOARDREPORTERSUBSYSTEM_H_ */

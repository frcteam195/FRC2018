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

#define DEFAULT_DASHBOARD_UDP_PORT 5804
#define DEFAULT_DASHBOARD_IP "10.1.95.14"

#include "Utilities/CustomSubsystem.h"
#include "Subsystems/DriveBaseSubsystem.h"
#include <Subsystems/CubeHandlerSubsystem.h>
#include "Utilities/GlobalDefines.h"

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
	DashboardReporterSubsystem(int udpPort);
	~DashboardReporterSubsystem();

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

	Controllers *robotControllers;
	DriveBaseSubsystem *driveBaseSubsystem;
	CubeHandlerSubsystem *cubeHandlerSubsystem;
	void runUDPSend();

	static DashboardReporterSubsystem *instance;

public:
	void init() override;
	void start() override;
	void stop() override;
	void subsystemHome() override {};

	static DashboardReporterSubsystem *getInstance();
	static DashboardReporterSubsystem *getInstance(vector<CustomSubsystem *> *subsystemVector);
};

#endif /* SRC_SUBSYSTEMS_DASHBOARDREPORTERSUBSYSTEM_H_ */

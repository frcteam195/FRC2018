/*
 * TuneablePID.h
 *
 *  Created on: Jan 10, 2017
 *      Author: roberthilton
 */

#ifndef SRC_UTILITIES_TUNEABLEPID_H_
#define SRC_UTILITIES_TUNEABLEPID_H_

#define BUFSIZE 2048
#define PID_RECEIVE_RATE_MS 40
#define PID_SEND_RATE_MS 10


#include <arpa/inet.h>
#include <iostream>
#include <mutex>
#include <netdb.h>
#include <sys/socket.h>
#include <time.h>
#include <cstring>
#include <thread>
#include <vector>


#include "ctre/Phoenix.h"
#include "WPILib.h"

#include "Constants.h"
#include "PIDValues.h"
#include "Utilities/GlobalDefines.h"
#include <vector>

using namespace std;

class TuneablePID {
private:
	string name;
	vector<TalonSRX *> *tuningVector;
	uint16_t portNumber;
	PIDValues pidValues;
	sockaddr_in localAddr; /* our address */
	sockaddr_in remoteAddr; /* remote address */
	socklen_t addrlen;

	double *setpointReq;

	int recvlen;
	int fd;
	unsigned char buf[BUFSIZE];

	int status;

	thread udpReceiveThread;
	thread udpSendThread;

	bool autoUpdate;
	bool autoUpdateSetpoint;
	bool runThread;
	mutex _tuneablePIDMutex;

	double pidSendThreadControlStart, pidSendThreadControlEnd;
	int pidSendThreadControlElapsedTimeMS;

	double pidReceiveThreadControlStart, pidReceiveThreadControlEnd;
	int pidReceiveThreadControlElapsedTimeMS;

	void runUDPReceive()
	{
		while (runThread)
		{
			pidReceiveThreadControlStart = Timer::GetFPGATimestamp();

			recvlen = recvfrom(fd, buf, BUFSIZE, 0, (sockaddr *) &remoteAddr, &addrlen);
			if (recvlen > 0) {
				buf[recvlen] = 0;
				_tuneablePIDMutex.lock();
				pidValues = processUDPPacket(buf, recvlen);
				_tuneablePIDMutex.unlock();
				//cout << "Running Tuneable PID for " << name << endl;
				for (unsigned int i = 0; i < tuningVector->size(); i++) {
					if (autoUpdate) {
						tuningVector->at(i)->Config_kP(0, pidValues.kP, kTimeoutMs);
						tuningVector->at(i)->Config_kI(0, pidValues.kI, kTimeoutMs);
						tuningVector->at(i)->Config_kD(0, pidValues.kD, kTimeoutMs);
						tuningVector->at(i)->Config_kF(0, pidValues.f, kTimeoutMs);
						if (tuningVector->at(i)->GetControlMode() == ControlMode::MotionMagic) {
							tuningVector->at(i)->ConfigMotionCruiseVelocity(pidValues.cruiseVelocity, kTimeoutMs);
							tuningVector->at(i)->ConfigMotionAcceleration(pidValues.acceleration, kTimeoutMs);
						}

						if (autoUpdateSetpoint)
							switch(tuningVector->at(i)->GetControlMode()) {
								case ControlMode::Velocity:
									tuningVector->at(i)->Set(tuningVector->at(i)->GetControlMode(), pidValues.setpoint * kSensorUnitsPerRotation / 600);
									break;
								default:
									tuningVector->at(i)->Set(tuningVector->at(i)->GetControlMode(), pidValues.setpoint);
									break;
							}
					}
				}



			} else
				;//cout << "uh oh - something went wrong!\n";

			//Ensure that we do not try to run faster than the sample period
			do {
				pidReceiveThreadControlEnd = Timer::GetFPGATimestamp();
				pidReceiveThreadControlElapsedTimeMS = (int) ((pidReceiveThreadControlEnd - pidReceiveThreadControlStart) * 1000);
				if (pidReceiveThreadControlElapsedTimeMS < PID_RECEIVE_RATE_MS)
					this_thread::sleep_for(chrono::milliseconds(PID_RECEIVE_RATE_MS - pidReceiveThreadControlElapsedTimeMS));
			} while(pidReceiveThreadControlElapsedTimeMS < PID_RECEIVE_RATE_MS);
		}

	};

	void runUDPSend()
	{
		while (runThread)
		{
			pidSendThreadControlStart = Timer::GetFPGATimestamp();

			double setpoint = *setpointReq;
			double actualValue = 0;
			int sensorSelect = 0;
			if (tuningVector->size() > 0) {
			//for (unsigned int i = 0; i < tuningVector->size(); i++) {
				switch(tuningVector->at(sensorSelect)->GetControlMode()) {
					case ControlMode::Position:
						actualValue = tuningVector->at(sensorSelect)->GetSelectedSensorPosition(0) / kSensorUnitsPerRotation;
						//setpoint = tuningVector->at(sensorSelect)->GetClosedLoopTarget(0) / kSensorUnitsPerRotation;
						break;
					case ControlMode::Current:
						actualValue = tuningVector->at(sensorSelect)->GetOutputCurrent();
						//setpoint = tuningVector->at(sensorSelect)->GetClosedLoopTarget(0);
						break;
					case ControlMode::Velocity:
						actualValue = tuningVector->at(sensorSelect)->GetSelectedSensorVelocity(0) / kSensorUnitsPerRotation * 600;
						//setpoint = tuningVector->at(sensorSelect)->GetClosedLoopTarget(0) / kSensorUnitsPerRotation;
						break;
					case ControlMode::MotionMagic:
						actualValue = tuningVector->at(sensorSelect)->GetSelectedSensorPosition(0) / kSensorUnitsPerRotation;
						//setpoint = tuningVector->at(sensorSelect)->GetClosedLoopTarget(0) / kSensorUnitsPerRotation;
						break;
					default:
						//setpoint = tuningVector->at(sensorSelect)->GetMotorOutputPercent();
						actualValue = tuningVector->at(sensorSelect)->GetSelectedSensorVelocity(0) / kSensorUnitsPerRotation * 600;
						break;
				}
			//}
			}

			if (autoUpdateSetpoint)
				setpoint = pidValues.setpoint;

			string sendStr = "Name:" + name + ";DesiredValue:" + to_string(setpoint) + ";ActualValue:" + to_string(actualValue) + ";";
			recvlen = sendto(fd, sendStr.c_str(), sendStr.length()+1, 0, (sockaddr *) &remoteAddr, addrlen);
			if (recvlen > 0) {
				;
			} else
				;//cout << "uh oh - something went wrong!\n";

			//Ensure that we do not try to run faster than the sample period
			do {
				pidSendThreadControlEnd = Timer::GetFPGATimestamp();
				pidSendThreadControlElapsedTimeMS = (int) ((pidSendThreadControlEnd - pidSendThreadControlStart) * 1000);
				if (pidSendThreadControlElapsedTimeMS < PID_SEND_RATE_MS)
					this_thread::sleep_for(chrono::milliseconds(PID_SEND_RATE_MS - pidSendThreadControlElapsedTimeMS));
			} while(pidSendThreadControlElapsedTimeMS < PID_SEND_RATE_MS);
		}

	};

	PIDValues processUDPPacket(unsigned char* buf, int recvlen) {

		PIDValues pidValues;

		for (unsigned int i = 0; i < strlen((const char*) buf); ++i)
			buf[i] = tolower(buf[i]);

		vector<char *> vS;
		char* chars_array = strtok((char*) buf, ";");
		while (chars_array) {
			vS.push_back(chars_array);
			chars_array = strtok(NULL, ";");
		}
		for (unsigned int i = 0; i < vS.size(); ++i) {
			char* cmd = strtok((char*) vS[i], ":");
			char* value = strtok(NULL, ":");
			if (strcmp(cmd, "kp") == 0)
				pidValues.kP = atof(value);
			else if (strcmp(cmd, "ki") == 0)
				pidValues.kI = atof(value);
			else if (strcmp(cmd, "kd") == 0)
				pidValues.kD = atof(value);
			else if (strcmp(cmd, "f") == 0)
				pidValues.f = atof(value);
			else if (strcmp(cmd, "setpoint") == 0)
				pidValues.setpoint = atof(value);
			else if (strcmp(cmd, "cruisevelocity") == 0)
				pidValues.cruiseVelocity = atof(value);
			else if (strcmp(cmd, "acceleration") == 0)
				pidValues.acceleration = atof(value);
		}
		return pidValues;
	}


public:
	/**
	 * TuneablePID used for calibrating PID systems over a UDP stream with our remote dashboard.
	 *
	 * @param name The name of the subsystem for display on the remote dashboard.
	 * @param tuningTalon The CANTalon to tune.
	 * @param portNumber The UDP port to listen and send on.
	 * @param autoUpdate Auto update the CANTalon. If set to true, will auto start threads. If false, threads must be started and stopped manually
	 */
	TuneablePID(string name, TalonSRX *tuningTalon, double *setpointReq, int portNumber, bool autoUpdate, bool autoUpdateSetpoint) { // @suppress("Class members should be properly initialized")
		vector<TalonSRX *> *tuningVector = new vector<TalonSRX *>();
		tuningVector->push_back(tuningTalon);
		this->name = name;
		this->tuningVector = tuningVector;
		this->portNumber = portNumber;
		this->autoUpdate = autoUpdate;
		this->autoUpdateSetpoint = autoUpdateSetpoint;
		this->setpointReq = setpointReq;
		initVars();
	};
	TuneablePID(string name, TalonSRX *tuningTalon, TalonSRX *tuningTalon2, double *setpointReq, int portNumber, bool autoUpdate, bool autoUpdateSetpoint) { // @suppress("Class members should be properly initialized")
		vector<TalonSRX *> *tuningVector = new vector<TalonSRX *>();
		tuningVector->push_back(tuningTalon);
		tuningVector->push_back(tuningTalon2);
		this->name = name;
		this->tuningVector = tuningVector;
		this->portNumber = portNumber;
		this->autoUpdate = autoUpdate;
		this->autoUpdateSetpoint = autoUpdateSetpoint;
		this->setpointReq = setpointReq;
		initVars();
	};
	TuneablePID(string name, vector<TalonSRX *> *tuningVector, double *setpointReq, int portNumber, bool autoUpdate, bool autoUpdateSetpoint) {
		this->name = name;
		this->tuningVector = tuningVector;
		this->portNumber = portNumber;
		this->autoUpdate = autoUpdate;
		this->autoUpdateSetpoint = autoUpdateSetpoint;
		this->setpointReq = setpointReq;
		initVars();
	};

	void initVars() {
		recvlen = 0;
		fd = 0;
		addrlen = 0;
		runThread = false;
		status = 0;
		//if (autoUpdate) {
			init();
			start();
		//}
		pidReceiveThreadControlStart = 0;
		pidReceiveThreadControlEnd = 0;
		pidReceiveThreadControlElapsedTimeMS = 0;

		pidSendThreadControlStart = 0;
		pidSendThreadControlEnd = 0;
		pidSendThreadControlElapsedTimeMS = 0;
	}

	~TuneablePID() {};

	void init()
	{
#ifdef TUNING_PIDS
		cout << "Init start for PID!\n";
		addrlen = sizeof(remoteAddr); /* length of addresses */

		if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
			cout << "cannot create socket\n";
			status = 1;
			return;
		}

		struct timeval tv;
		tv.tv_sec = 0;
		tv.tv_usec = 100000;
		if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0)
		    cout << "Error setting socket timeout" << endl;

		/* bind the socket to any valid IP address and a specific port */

		memset((char *) &localAddr, 0, sizeof(localAddr));
		localAddr.sin_family = AF_INET;
		localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
		localAddr.sin_port = htons(portNumber);

		if (portNumber < 5805) {
			status = 3;
			cout << "Invalid port number for PID Tuner " << name << "! Valid port numbers are 5805-5809." << endl;
			return;
		}

		if (bind(fd, (sockaddr *) &localAddr, sizeof(localAddr)) < 0) {
			cout << "bind failed";
			status = 2;
			return;
		}
		cout << "Init finish for PID!\n";
#endif
		status = 0;
	};

	/**
	 * Start the threads running ifdef TUNING_PIDS
	 */
	void start() {
#ifdef TUNING_PIDS
		if (status != 0)
			return;

		runThread = true;
		udpReceiveThread = thread(&TuneablePID::runUDPReceive, this);
		udpSendThread = thread(&TuneablePID::runUDPSend, this);
#endif
	};

	/**
	 * Stop the threads running ifdef TUNING_PIDS
	 */
	void stop() {
#ifdef TUNING_PIDS
		if (status != 0)
			return;

		runThread = false;
		if (udpReceiveThread.joinable())
			udpReceiveThread.join();
		if (udpSendThread.joinable())
			udpSendThread.join();
#endif
	};

	/**
	 * Gets PIDValue container from this TuneablePID
	 * @return Struct of PIDValues
	 */
	PIDValues getPIDValues() {
		return pidValues;
	};

	bool isAutoUpdate() {
		return autoUpdate;
	};
};



#endif /* SRC_UTILITIES_TUNEABLEPID_H_ */

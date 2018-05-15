#ifndef KNIGHTVISION_UDPSOCKET_H_
#define KNIGHTVISION_UDPSOCKET_H_

#define BUFSIZE 2048
#define PID_RECEIVE_RATE_MS 10
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

#include "SimpleGPIO.h"

using namespace std;

class UDPSocket {
public:
	UDPSocket(int portNumber);
	~UDPSocket() {}

	void init();
	void start();
	void stop();

	void updateValues(double deviation, double targetDistance, bool onTarget);
	void udpSendSynchronous(double deviation, double targetDistance, bool onTarget, bool targetFound, unsigned long sequence);
private:
	double deviation;
	double targetDistance;
	bool onTarget;
	string strOnTarget;
	
	const unsigned int LEDGPIO = 57;

	uint16_t portNumber;
	sockaddr_in localAddr; /* our address */
	sockaddr_in remoteAddr; /* remote address */
	socklen_t addrlen;

	int recvlenReceive;
	int recvlenSend;
	int fd;
	unsigned char buf[BUFSIZE];

	int status;

	thread udpReceiveThread;
	thread udpSendThread;

	bool autoUpdate;
	bool autoUpdateSetpoint;
	bool runThread;
	bool ledState;
	mutex _mutex;

	void runUDPReceive();
	void runUDPSend();
	void processUDPPacket(unsigned char* buf, int recvlen);
};

#endif /* KNIGHTVISION_UDPSOCKET_H_ */

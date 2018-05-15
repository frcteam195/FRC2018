#include "UDPSocket.h"

UDPSocket::UDPSocket(int portNumber) {
	deviation = 0;

	this->portNumber = portNumber;
	recvlenReceive = 0;
	recvlenSend = 0;
	fd = 0;
	addrlen = 0;
	runThread = false;
	ledState = false;
	status = 0;
}

void UDPSocket::init() {
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

	/* bind the socket to any valid IP address and a specific port */
	memset((char *) &localAddr, 0, sizeof(localAddr));
	localAddr.sin_family = AF_INET;
	localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	//localAddr.sin_port = htons(0);
	localAddr.sin_port = htons(portNumber);

	remoteAddr.sin_family = AF_INET;
	//remoteAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	//remoteAddr.sin_port = htons(0);
	inet_aton("10.1.95.22", &(remoteAddr.sin_addr));
	remoteAddr.sin_port = htons(portNumber);

	if (portNumber > 5804) {
		status = 3;
		cout << "Invalid port number! Valid port numbers are 5805-5809." << endl;
		return;
	}

	if (::bind(fd, (sockaddr *) &localAddr, sizeof(localAddr)) < 0) {
		cout << "bind failed";
		status = 2;
		return;
	}
	cout << "Init finish for Send!\n";
	status = 0;
}

void UDPSocket::start() {
	if (status != 0)
		return;

	runThread = true;
	udpReceiveThread = thread(&UDPSocket::runUDPReceive, this);
	//udpSendThread = thread(&UDPSocket::runUDPSend, this);
}

void UDPSocket::stop() {
	if(status != 0)
		return;

	runThread = false;
	if(udpReceiveThread.joinable())
		udpReceiveThread.join();
	if(udpSendThread.joinable())
		udpSendThread.join();
}

void UDPSocket::updateValues(double deviation, double targetDistance, bool onTarget) {
	_mutex.lock();
	this->deviation = deviation;
	this->targetDistance = targetDistance;
	this->onTarget = onTarget;
	_mutex.unlock();
}

void UDPSocket::runUDPReceive() {
	while (runThread) {
		chrono::time_point<std::chrono::system_clock> start, end;
		chrono::duration<double> elapsedSeconds;
		int elapsedTimeMS;
		start = chrono::system_clock::now();
		//cout << "waiting for packet!" << endl;
		recvlenReceive = recvfrom(fd, buf, BUFSIZE, 0, (sockaddr *) &remoteAddr, &addrlen);
		if (recvlenReceive > 0) {
			buf[recvlenReceive] = 0;
			//cout << "Got Packet from RIO!" << endl;
			processUDPPacket(buf, recvlenReceive);
		} else
			;//cout << "uh oh - something went wrong!\n";

		
		do {
			end = chrono::system_clock::now();
			elapsedSeconds = end - start;
			elapsedTimeMS = (int)(elapsedSeconds.count()*1000);
			if (elapsedTimeMS < PID_RECEIVE_RATE_MS)
				this_thread::sleep_for(chrono::milliseconds(PID_RECEIVE_RATE_MS - elapsedTimeMS));
		} while(elapsedTimeMS < PID_RECEIVE_RATE_MS);
	}
}

void UDPSocket::runUDPSend() {
	while(runThread) {
		chrono::time_point<std::chrono::system_clock> start, end;
		chrono::duration<double> elapsedSeconds;
		int elapsedTimeMS;
		start = chrono::system_clock::now();

		if (onTarget)
			strOnTarget = "true";
		else
			strOnTarget = "false";
		
		string sendStr = "AngleDeviation:" + to_string(deviation) + ";TargetDistance:" + to_string(targetDistance) + ";OnTarget:" + strOnTarget + ";";
		recvlenSend = sendto(fd, sendStr.c_str(), sendStr.length()+1, 0, (sockaddr *) &remoteAddr, addrlen);
		if (recvlenSend > 0) {
			//cout << "Packet Sent!" << endl;;
		} else {}
			//cout << "uh oh - something went wrong!\n";
			//cout << "Deviation: " << deviation << endl;
			

		//Ensure that we do not try to run faster than the sample period
		do {
			end = chrono::system_clock::now();
			elapsedSeconds = end - start;
			elapsedTimeMS = (int)(elapsedSeconds.count()*1000);
			if (elapsedTimeMS < PID_SEND_RATE_MS)
				this_thread::sleep_for(chrono::milliseconds(PID_SEND_RATE_MS - elapsedTimeMS));
		} while(elapsedTimeMS < PID_SEND_RATE_MS);
	}
}

void UDPSocket::processUDPPacket(unsigned char* buf, int recvlen) {
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
		if (strcmp(cmd, "visionenable") == 0) {
			if (strcmp(value, "true") == 0) {
				if (!ledState) {
					gpio_set_value(LEDGPIO, PIN_VALUE::HIGH);
					ledState = true;
				}
			}
			else {
				if (ledState) {
					gpio_set_value(LEDGPIO, PIN_VALUE::LOW);
					ledState = false;
				}
			}
		}
	}
}

void UDPSocket::udpSendSynchronous(double deviation, double targetDistance, bool onTarget, bool targetFound, unsigned long sequence) {
	//if (ledState) {
		string strOnTarget = "";
		if (onTarget)
			strOnTarget = "true";
		else
			strOnTarget = "false";
	
		string strTargetFound = "";
		if (targetFound)
			strTargetFound = "true";
		else
			strTargetFound = "false";
	
                string sendStr = "AngleDeviation:" + to_string(deviation) + ";TargetDistance:" + to_string(targetDistance) + ";OnTarget:" + strOnTarget + ";TargetFound:" + strTargetFound + ";Sequence:" + to_string(sequence) + ";";
                recvlenSend = sendto(fd, sendStr.c_str(), sendStr.length()+1, 0, (sockaddr *) &remoteAddr, addrlen);
                if (recvlenSend > 0) {
                        //cout << "Packet Sent!" << + sequence << endl;;
                } else {}
                        //cout << "uh oh - something went wrong!\n";
                        //cout << "Deviation: " << deviation << endl;
	//}
}

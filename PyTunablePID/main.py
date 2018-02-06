import UDPComm

if __name__ == '__main__':
	socket = UDPComm.UDPComm('10.1.95.22', 5808)
	socket.connect()
	try:
		while True:
			socket.sendPacket()
			#print(socket.recvPacket())
	except KeyboardInterrupt:
		socket.quitSocket()

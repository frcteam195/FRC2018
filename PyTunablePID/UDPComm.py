import socket

class UDPComm:
	def __init__(self, ip='10.1.95.22', port=5801):
		self.ip = str(ip)
		self.port = int(port)
		self.udpSendSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
		self.udpRecvSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
		#self.udpRecvSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.pidValues = {'p': 0.0, 'i': 0.0, 'd': 0.0, 'setpoint': 0.0, 'actual': 0.0}

	def connect(self):
		self.udpRecvSock.bind(('127.0.0.1', self.port))

	def sendPacket(self, p=0.0, i=0.0, d=0.0, f=0.0, ramprate=0.0, izone=0.0, setpoint=0.0, cruisevelocity=0.0, acceleration=0.0):
		message = 'kp:' + str(p) + ';ki:' + str(i) + ';kd:' + str(d) + ';f:' + str(f) + ';ramprate:' + str(ramprate) + ';izone:' + str(izone) + ';setpoint:' + str(setpoint) + ';cruisevelocity:' + str(cruisevelocity) + ';acceleration:' + str(acceleration) + ';'
		self.udpSendSock.sendto(message.encode(), (self.ip, self.port))

	def recvPacket(self):
		data, addr = self.udpRecvSock.recvfrom(1024)
		return data.decode()

	def processPacket(self, data):
		while ':' in data and  ';' in data:
			title = data[0:data.index(':')].lower()
			value = data[data.index(':')+1:data.index(';')]
			data = data[data.index(';')+1:]

			if title == 'p':
				self.pidValues['p'] = float(value)
			elif title == 'i':
				self.pidValues['i'] = float(value)
			elif title == 'd':
				self.pidValues['d'] = float(value)
			elif title == 'setpoint':
				self.pidValues['setpoint'] = float(value)
			elif title == 'actual':
				self.pidValues['actual'] = float(value)

		return self.pidValues

	def quitSocket(self):
		if self.udpRecvSock:
			self.udpRecvSock.close()
		if self.udpSendSock:
			self.udpSendSock.close()

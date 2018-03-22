import socket
import PIDValues
import ResponseData

class UDPComm:
	OBJ_IP_ALL = '0.0.0.0'

	def __init__(self, ip='10.1.95.22', port=5801):
		self.ip = str(ip)
		self.port = int(port)
		self.udpSendSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
		self.udpRecvSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
		self.udpRecvSock.settimeout(1)

	def connect(self):
		self.udpRecvSock.bind((self.OBJ_IP_ALL, self.port))

	def close(self):
		if self.udpRecvSock:
			self.udpRecvSock.close()
		if self.udpSendSock:
			self.udpSendSock.close()

	def getResponseData(self):
		return self.__processPacket(self.__recvPacket())

	def sendPIDData(self, pid=PIDValues.PIDValues()):
		if isinstance(pid, PIDValues.PIDValues):
			message = 'kp:' + str(pid.p) + ';'
			message += 'ki:' + str(pid.i) + ';'
			message += 'kd:' + str(pid.d) + ';'
			message += 'f:' + str(pid.f) + ';'
			message += 'ramprate:' + str(pid.ramprate) + ';'
			message += 'izone:' + str(pid.izone) + ';'
			message += 'setpoint:' + str(pid.setpoint) + ';'
			message += 'cruisevelocity:' + str(pid.cruisevelocity) + ';'
			message += 'acceleration:' + str(pid.acceleration) + ';'
			self.__sendPacket(message)

	def __sendPacket(self, message):
		self.udpSendSock.sendto(message.encode(), (self.ip, self.port))

	def __recvPacket(self):
		try :
			data, addr = self.udpRecvSock.recvfrom(1024)
			return data.decode()
		except socket.timeout:
			return ""

	def __processPacket(self, data=""):
		while ':' in data and  ';' in data:
			title = data[0:data.index(':')].lower()
			value = data[data.index(':')+1:data.index(';')]
			data = data[data.index(';')+1:]

			responseData = ResponseData.ResponseData()

			if title == 'name':
				responseData.name = str(value)
			elif title == 'desiredvalue':
				responseData.desiredValue = float(value)
			elif title == 'actualvalue':
				responseData.actualValue = float(value)
			elif title == 'integralaccum':
				responseData.integralAccum = float(value)

		return responseData

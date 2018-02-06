import UDPComm
import PIDValues

if __name__ == '__main__':
	udpComm = UDPComm.UDPComm('10.1.95.22', 5808)
	udpComm.connect()
	try:
		while True:
			udpComm.sendPIDData(PIDValues.PIDValues(2, 1, 2, 3, 4))
			print(udpComm.getResponseData())
	except KeyboardInterrupt:
		udpComm.close()

import serial
import time
import math
import numpy as np

LOBOT_SERVO_FRAME_HEADER         =0x55
LOBOT_SERVO_MOVE_TIME_WRITE      =1
LOBOT_SERVO_MOVE_TIME_READ       =2
LOBOT_SERVO_MOVE_TIME_WAIT_WRITE =7
LOBOT_SERVO_MOVE_TIME_WAIT_READ  =8
LOBOT_SERVO_MOVE_START           =11
LOBOT_SERVO_MOVE_STOP            =12
LOBOT_SERVO_ID_WRITE             =13
LOBOT_SERVO_ID_READ              =14
LOBOT_SERVO_ANGLE_OFFSET_ADJUST  =17
LOBOT_SERVO_ANGLE_OFFSET_WRITE   =18
LOBOT_SERVO_ANGLE_OFFSET_READ    =19
LOBOT_SERVO_ANGLE_LIMIT_WRITE    =20
LOBOT_SERVO_ANGLE_LIMIT_READ     =21
LOBOT_SERVO_VIN_LIMIT_WRITE      =22
LOBOT_SERVO_VIN_LIMIT_READ       =23
LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE =24
LOBOT_SERVO_TEMP_MAX_LIMIT_READ  =25
LOBOT_SERVO_TEMP_READ            =26
LOBOT_SERVO_VIN_READ             =27
LOBOT_SERVO_POS_READ             =28
LOBOT_SERVO_OR_MOTOR_MODE_WRITE  =29
LOBOT_SERVO_OR_MOTOR_MODE_READ   =30
LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE =31
LOBOT_SERVO_LOAD_OR_UNLOAD_READ  =32
LOBOT_SERVO_LED_CTRL_WRITE       =33
LOBOT_SERVO_LED_CTRL_READ        =34
LOBOT_SERVO_LED_ERROR_WRITE      =35
LOBOT_SERVO_LED_ERROR_READ       =36

class Actuator_Interface():
	def __init__(self,port_name,baud_rate,ID_list,calibration=None,debug=False):
		self.ser = serial.Serial(port_name,baud_rate,timeout=1)
		self.ID_list = ID_list
		if calibration is not None:
			self.calibration = calibration ## a tuple of two np arrays (4,3) of offsets and scaling factors (in counts)
		else:
			self.run_calibration()

		if debug:
			while True:
				print(self.get_actuator_counts())
				time.sleep(0.1)

	def run_calibration(self):
		print("Running Actuator Calibration!")
		input("Bring all actuators to nominal 0 position...press enter when finished")
		offsets = np.reshape(self.get_actuator_counts(),(4,3))
		print("OFFSETS: ", offsets)
		input("Bring all actuators to nominal +/-90deg position...press enter when finished")
		scaling = np.absolute((np.reshape(self.get_actuator_counts(),(4,3))-offsets))/(math.pi/2)
		print("SCALING: ", scaling)
		self.calibration = (offsets,scaling)

	def get_actuator_angs(self):
		return self.counts_to_angs(np.reshape(self.get_actuator_counts(),(4,3)))

	def angs_to_counts(self,angs):
		return angs*self.calibration[1]+self.calibration[0]

	def counts_to_angs(self,counts):
		return (counts-self.calibration[0])/self.calibration[1]

	def get_actuator_counts(self):
		positions = []
		for ID in self.ID_list:
			positions.append(self.LobotSerialServoReadPosition(ID))
		return positions

	def stop_movement(self):
		for ID in self.ID_list:
			self.LobotSerialServoMoveStop(ID)

	def unload_servos(self):
		for ID in self.ID_list:
			self.LobotSerialServoWriteLoadOrUnload(ID,0)
			print("UNLOADED ID: ",ID)

	def checkSum(self,buf):
		check = 0
		for i in range(2,buf[3]+2):
			check+=buf[i]
		check = ~check&0xFF
		return check

	def LobotSerialServoReceiveHandle(self):
		frameStarted = False
		receiveFinished = False
		frameCount = 0
		dataCount = 0
		dataLength = -10
		recvBuf = [None]*32
		while self.ser.in_waiting:
			rxBuf = self.ser.read(size=1)
			if len(rxBuf)==0:
				continue
			else:
				rxBuf = ord(rxBuf)
			time.sleep(0.0001)
			if not frameStarted:
				if rxBuf == 0x55:
					frameCount+=1
					if frameCount==2:
						frameStarted = True
				else:
					frameStarted = False
					frameCount = 0
			else:
				recvBuf[dataCount+2] = rxBuf
				if dataCount == 3:
					dataLength = recvBuf[dataCount]
					if dataLength<3 or dataCount>7:
						dataLength = 2
						frameStarted = False
				dataCount+=1    
				if dataCount == dataLength+1:
					if self.checkSum(recvBuf)==recvBuf[dataCount+1]:
						frameStarted = False
						return 1, recvBuf
					return -1, recvBuf
		return -1, recvBuf

	def LobotSerialServoReadPosition(self, ID):
		count = 10000
		buf = [0x55,0x55]
		buf.append(ID)
		buf.append(3)
		buf.append(LOBOT_SERVO_POS_READ)
		buf.append(self.checkSum(buf))
		while self.ser.in_waiting:
			self.ser.read(size=1)
		self.ser.write(serial.to_bytes(buf))
		while not self.ser.in_waiting:
			count-=1
			if count<0:
				return -2048
		status,msg = self.LobotSerialServoReceiveHandle()
		if status > 0:
			msg = (msg[6]<<8)|msg[5]
		else:
			msg = -2048
		return msg #returns in counts

	def LobotSerialServoReadTemp(self, ID):
		count = 10000
		buf = [0x55,0x55]
		buf.append(ID)
		buf.append(3)
		buf.append(LOBOT_SERVO_TEMP_READ)
		buf.append(self.checkSum(buf))
		while self.ser.in_waiting:
			self.ser.read(size=1)
		self.ser.write(serial.to_bytes(buf))
		while not self.ser.in_waiting:
			count-=1
			if count<0:
				return -2048
		status,msg = self.LobotSerialServoReceiveHandle()
		if status > 0:
			msg = msg[5]
		else:
			msg = -2048
		return msg #returns in deg C

	def LobotSerialServoReadError(self, ID):
		count = 10000
		buf = [0x55,0x55]
		buf.append(ID)
		buf.append(3)
		buf.append(LOBOT_SERVO_LED_ERROR_READ)
		buf.append(self.checkSum(buf))
		while self.ser.in_waiting:
			self.ser.read(size=1)
		self.ser.write(serial.to_bytes(buf))
		while not self.ser.in_waiting:
			count-=1
			if count<0:
				return -2048
		status,msg = self.LobotSerialServoReceiveHandle()
		if status > 0:
			msg = msg[5]
		else:
			msg = -2048
		return msg #returns error code 0-7

	def LobotSerialServoReadMaxTempLimit(self, ID):
		count = 10000
		buf = [0x55,0x55]
		buf.append(ID)
		buf.append(3)
		buf.append(LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
		buf.append(self.checkSum(buf))
		while self.ser.in_waiting:
			self.ser.read(size=1)
		self.ser.write(serial.to_bytes(buf))
		while not self.ser.in_waiting:
			count-=1
			if count<0:
				return -2048
		status,msg = self.LobotSerialServoReceiveHandle()
		if status > 0:
			msg = msg[5]
		else:
			msg = -2048
		return msg #returns in deg C

	def LobotSerialServoReadVinLimits(self, ID):
		count = 10000
		buf = [0x55,0x55]
		buf.append(ID)
		buf.append(3)
		buf.append(LOBOT_SERVO_VIN_LIMIT_READ)
		buf.append(self.checkSum(buf))
		while self.ser.in_waiting:
			self.ser.read(size=1)
		self.ser.write(serial.to_bytes(buf))
		while not self.ser.in_waiting:
			count-=1
			if count<0:
				return -2048
		status,msg = self.LobotSerialServoReceiveHandle()
		if status > 0:
			msg = [(msg[6]<<8)|msg[5],(msg[8]<<8)|msg[7]]
		else:
			msg = -2048
		return msg #returns in mV

	def LobotSerialServoReadAngleLimits(self, ID):
		count = 10000
		buf = [0x55,0x55]
		buf.append(ID)
		buf.append(3)
		buf.append(LOBOT_SERVO_ANGLE_LIMIT_READ)
		buf.append(self.checkSum(buf))
		while self.ser.in_waiting:
			self.ser.read(size=1)
		self.ser.write(serial.to_bytes(buf))
		while not self.ser.in_waiting:
			count-=1
			if count<0:
				return -2048
		status,msg = self.LobotSerialServoReceiveHandle()
		if status > 0:
			msg = [(msg[6]<<8)|msg[5],(msg[8]<<8)|msg[7]]
		else:
			msg = -2048
		return msg #returns in counts

	def LobotSerialServoReadVin(self, ID):
		count = 10000
		buf = [0x55,0x55]
		buf.append(ID)
		buf.append(3)
		buf.append(LOBOT_SERVO_VIN_READ)
		buf.append(self.checkSum(buf))
		while self.ser.in_waiting:
			self.ser.read(size=1)
		self.ser.write(serial.to_bytes(buf))
		while not self.ser.in_waiting:
			count-=1
			if count<0:
				return -2048
		status,msg = self.LobotSerialServoReceiveHandle()
		if status > 0:
			msg = (msg[6]<<8)|msg[5]
		else:
			msg = -2048
		return msg #returns in mV

	def LobotSerialServoMove(self,ID,pos,time): #pos in counts, time in ms
		buf = [0x55,0x55]
		if pos < 0:
			pos = 0
		if pos > 1000:
			pos = 1000
		buf.append(ID)
		buf.append(7)
		buf.append(LOBOT_SERVO_MOVE_TIME_WRITE)
		buf.append(pos&0xFF)
		buf.append(pos>>8)
		buf.append(time&0xFF)
		buf.append(time>>8)
		buf.append(self.checkSum(buf))
		self.ser.write(serial.to_bytes(buf))

	def LobotSerialServoWriteAngleLimits(self,ID,limits): #pos in counts, time in ms
		minAng = limits[0]
		maxAng = limits[1]
		buf = [0x55,0x55]
		if minAng < 0:
			minAng = 0
		if minAng > 1000:
			minAng = 1000
		if maxAng < 0:
			maxAng = 0
		if maxAng > 1000:
			maxAng = 1000
		if minAng>maxAng:
			maxAngCopy = maxAng
			maxAng = minAng
			minAng = maxAngCopy
		buf.append(ID)
		buf.append(7)
		buf.append(LOBOT_SERVO_ANGLE_LIMIT_WRITE)
		buf.append(minAng&0xFF)
		buf.append(minAng>>8)
		buf.append(maxAng&0xFF)
		buf.append(maxAng>>8)
		buf.append(self.checkSum(buf))
		self.ser.write(serial.to_bytes(buf))

	def LobotSerialServoWriteMaxTempLimit(self,ID,temp): #ID int in 1-255, temp in deg C
		buf = [0x55,0x55]
		buf.append(ID)
		buf.append(4)
		buf.append(LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE)
		buf.append(temp)
		buf.append(self.checkSum(buf))
		self.ser.write(serial.to_bytes(buf))

	def LobotSerialServoWriteLoadOrUnload(self,ID,state): #ID int in 1-255, state is 0 for unload, 1 for load
		buf = [0x55,0x55]
		buf.append(ID)
		buf.append(4)
		buf.append(LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE)
		buf.append(state)
		buf.append(self.checkSum(buf))
		self.ser.write(serial.to_bytes(buf))

	def LobotSerialServoMoveStop(self,ID): #ID int in 1-255, state is 0 for unload, 1 for load
		buf = [0x55,0x55]
		buf.append(ID)
		buf.append(3)
		buf.append(LOBOT_SERVO_MOVE_STOP)
		buf.append(self.checkSum(buf))
		self.ser.write(serial.to_bytes(buf))

	def LobotSerialWriteLEDError(self,ID,code): #ID int in 1-255, code int in 0-7 ##USE THIS TO RESET ERROR CODE TO 0###
		buf = [0x55,0x55]
		buf.append(ID)
		buf.append(4)
		buf.append(LOBOT_SERVO_LED_ERROR_WRITE)
		buf.append(code)
		buf.append(self.checkSum(buf))
		self.ser.write(serial.to_bytes(buf))

	def LobotSerialServoSetID(self,oldID,newID): #oldID int in 1-255, newID int in 1-255
		buf = [0x55,0x55]
		buf.append(oldID)
		buf.append(4)
		buf.append(LOBOT_SERVO_ID_WRITE)
		buf.append(newID)
		buf.append(self.checkSum(buf))
		self.ser.write(serial.to_bytes(buf))

if __name__ == '__main__':
	pass


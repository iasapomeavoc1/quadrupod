#!/usr/bin/env python

import serial
import time
import math

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

def checkSum(buf):
	check = 0
	for i in range(2,buf[3]+2):
		check+=buf[i]
	check = ~check&0xFF
	return check

def LobotSerialServoReceiveHandle(ser):
	frameStarted = False
	receiveFinished = False
	frameCount = 0
	dataCount = 0
	dataLength = -10
	recvBuf = [None]*32
	while ser.in_waiting:
		rxBuf = ord(ser.read(size=1))
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
				if checkSum(recvBuf)==recvBuf[dataCount+1]:
					frameStarted = False
					return 1, recvBuf
				return -1, recvBuf
	return -1, recvBuf

def LobotSerialServoReadPosition(ser, ID):
	count = 10000
	buf = [0x55,0x55]
	buf.append(ID)
	buf.append(3)
	buf.append(LOBOT_SERVO_POS_READ)
	buf.append(checkSum(buf))
	while ser.in_waiting:
		ser.read(size=1)
	ser.write(serial.to_bytes(buf))
	while not ser.in_waiting:
		count-=1
		if count<0:
			return -2048
	status,msg = LobotSerialServoReceiveHandle(ser)
	if status > 0:
		msg = (msg[6]<<8)|msg[5]
	else:
		msg = -2048
	return msg #returns in counts

def LobotSerialServoReadTemp(ser, ID):
	count = 10000
	buf = [0x55,0x55]
	buf.append(ID)
	buf.append(3)
	buf.append(LOBOT_SERVO_TEMP_READ)
	buf.append(checkSum(buf))
	while ser.in_waiting:
		ser.read(size=1)
	ser.write(serial.to_bytes(buf))
	while not ser.in_waiting:
		count-=1
		if count<0:
			return -2048
	status,msg = LobotSerialServoReceiveHandle(ser)
	if status > 0:
		msg = msg[5]
	else:
		msg = -2048
	return msg #returns in deg C

def LobotSerialServoReadError(ser, ID):
	count = 10000
	buf = [0x55,0x55]
	buf.append(ID)
	buf.append(3)
	buf.append(LOBOT_SERVO_LED_ERROR_READ)
	buf.append(checkSum(buf))
	while ser.in_waiting:
		ser.read(size=1)
	ser.write(serial.to_bytes(buf))
	while not ser.in_waiting:
		count-=1
		if count<0:
			return -2048
	status,msg = LobotSerialServoReceiveHandle(ser)
	if status > 0:
		msg = msg[5]
	else:
		msg = -2048
	return msg #returns error code 0-7

def LobotSerialServoReadMaxTempLimit(ser, ID):
	count = 10000
	buf = [0x55,0x55]
	buf.append(ID)
	buf.append(3)
	buf.append(LOBOT_SERVO_TEMP_MAX_LIMIT_READ)
	buf.append(checkSum(buf))
	while ser.in_waiting:
		ser.read(size=1)
	ser.write(serial.to_bytes(buf))
	while not ser.in_waiting:
		count-=1
		if count<0:
			return -2048
	status,msg = LobotSerialServoReceiveHandle(ser)
	if status > 0:
		msg = msg[5]
	else:
		msg = -2048
	return msg #returns in deg C

def LobotSerialServoReadVinLimits(ser, ID):
	count = 10000
	buf = [0x55,0x55]
	buf.append(ID)
	buf.append(3)
	buf.append(LOBOT_SERVO_VIN_LIMIT_READ)
	buf.append(checkSum(buf))
	while ser.in_waiting:
		ser.read(size=1)
	ser.write(serial.to_bytes(buf))
	while not ser.in_waiting:
		count-=1
		if count<0:
			return -2048
	status,msg = LobotSerialServoReceiveHandle(ser)
	if status > 0:
		msg = [(msg[6]<<8)|msg[5],(msg[8]<<8)|msg[7]]
	else:
		msg = -2048
	return msg #returns in mV

def LobotSerialServoReadAngleLimits(ser, ID):
	count = 10000
	buf = [0x55,0x55]
	buf.append(ID)
	buf.append(3)
	buf.append(LOBOT_SERVO_ANGLE_LIMIT_READ)
	buf.append(checkSum(buf))
	while ser.in_waiting:
		ser.read(size=1)
	ser.write(serial.to_bytes(buf))
	while not ser.in_waiting:
		count-=1
		if count<0:
			return -2048
	status,msg = LobotSerialServoReceiveHandle(ser)
	if status > 0:
		msg = [(msg[6]<<8)|msg[5],(msg[8]<<8)|msg[7]]
	else:
		msg = -2048
	return msg #returns in counts

def LobotSerialServoReadVin(ser, ID):
	count = 10000
	buf = [0x55,0x55]
	buf.append(ID)
	buf.append(3)
	buf.append(LOBOT_SERVO_VIN_READ)
	buf.append(checkSum(buf))
	while ser.in_waiting:
		ser.read(size=1)
	ser.write(serial.to_bytes(buf))
	while not ser.in_waiting:
		count-=1
		if count<0:
			return -2048
	status,msg = LobotSerialServoReceiveHandle(ser)
	if status > 0:
		msg = (msg[6]<<8)|msg[5]
	else:
		msg = -2048
	return msg #returns in mV

def LobotSerialServoMove(ser,ID,pos,time): #pos in counts, time in ms
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
	buf.append(checkSum(buf))
	ser.write(serial.to_bytes(buf))

def LobotSerialServoWriteAngleLimits(ser,ID,limits): #pos in counts, time in ms
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
	buf.append(checkSum(buf))
	ser.write(serial.to_bytes(buf))

def LobotSerialServoWriteMaxTempLimit(ser,ID,temp): #ID int in 1-255, temp in deg C
	buf = [0x55,0x55]
	buf.append(ID)
	buf.append(4)
	buf.append(LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE)
	buf.append(temp)
	buf.append(checkSum(buf))
	ser.write(serial.to_bytes(buf))

def LobotSerialServoWriteLoadOrUnload(ser,ID,state): #ID int in 1-255, state is 0 for unload, 1 for load
	buf = [0x55,0x55]
	buf.append(ID)
	buf.append(4)
	buf.append(LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE)
	buf.append(state)
	buf.append(checkSum(buf))
	ser.write(serial.to_bytes(buf))

def LobotSerialServoMoveStop(ser,ID): #ID int in 1-255, state is 0 for unload, 1 for load
	buf = [0x55,0x55]
	buf.append(ID)
	buf.append(3)
	buf.append(LOBOT_SERVO_MOVE_STOP)
	buf.append(checkSum(buf))
	ser.write(serial.to_bytes(buf))

def LobotSerialWriteLEDError(ser,ID,code): #ID int in 1-255, code int in 0-7 ##USE THIS TO RESET ERROR CODE TO 0###
	buf = [0x55,0x55]
	buf.append(ID)
	buf.append(4)
	buf.append(LOBOT_SERVO_LED_ERROR_WRITE)
	buf.append(code)
	buf.append(checkSum(buf))
	ser.write(serial.to_bytes(buf))

def LobotSerialServoSetID(ser,oldID,newID): #oldID int in 1-255, newID int in 1-255
	buf = [0x55,0x55]
	buf.append(oldID)
	buf.append(4)
	buf.append(LOBOT_SERVO_ID_WRITE)
	buf.append(newID)
	buf.append(checkSum(buf))
	ser.write(serial.to_bytes(buf))

def get_actuator_positions(ser,IDs):
	positions = []
	for ID in IDs:
		positions.append(LobotSerialServoReadPosition(ser,ID))
	return positions

def stop_movement(ser,IDs):
	for ID in IDs:
		LobotSerialServoMoveStop(ser,ID)

def unload_servos(ser,IDs):
	for ID in IDs:
		LobotSerialServoWriteLoadOrUnload(ser,ID,0)

if __name__ == '__main__':
	#pass
	lx16a_port = serial.Serial('/dev/ttyS0',115200,timeout=1)
	lx16a_ids = (1,2,3,4,5,6,7,8,9,10,11,12)
	lx16a_offsets = [521.0,295.0,707.0,
							 484.0,222.0,656.0,
							 495.0,343.0,609.0,
							 483.0,205.0,656.0]
	lx16a_scaling_coeffs = [250.0,250.0,240.0,
									250.0,240.0,240.0,
									240.0,250.0,240.0,
									 240.0,250.0,240.0]
	commands = [0,0,0,0,0,0,0,0,0,0,0,0]
	converted = []
	for i in range(len(lx16a_ids)):
		print(lx16a_ids[i],":",LobotSerialServoReadAngleLimits(lx16a_port,lx16a_ids[i]))
		converted.append(commands[i]*lx16a_scaling_coeffs[i]+lx16a_offsets[i])
	
	#for i in range(len(lx16a_ids)):
	#	LobotSerialServoMove(lx16a_port,lx16a_ids[i],int(converted[i]),1000)
	
	while True:
		try:
			print("positions:",get_actuator_positions(lx16a_port,lx16a_ids))
			print("commands:",converted)
		except KeyboardInterrupt:
			stop_movement(lx16a_port,lx16a_ids)
			unload_servos(lx16a_port,lx16a_ids)
			exit()

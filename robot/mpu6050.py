#!/usr/bin/python
import smbus2
import math
from os import system
import time

def read_byte(reg):
	return bus.read_byte_data(address, reg)
 
def read_word(reg):
	h = bus.read_byte_data(address, reg)
	l = bus.read_byte_data(address, reg+1)
	value = (h << 8) + l
	return value
 
def read_word_2c(reg):
	val = read_word(reg)
	if (val >= 0x8000):
		return -((65535 - val) + 1)
	else:
		return val
 
def dist(a,b):
	return math.sqrt((a*a)+(b*b))
 
def get_y_rotation(x,y,z):
	radians = math.atan2(x, dist(y,z))
	return -math.degrees(radians)
 
def get_x_rotation(x,y,z):
	radians = math.atan2(y, dist(x,z))
	return math.degrees(radians)

if __name__ == '__main__':
	while True:
		system('clear')

		bus.write_byte_data(address, power_mgmt_1, 0)
		
		print("Gyro")
		print("--------")
		 
		gyro_xout = read_word_2c(0x43)
		gyro_yout = read_word_2c(0x45)
		gyro_zout = read_word_2c(0x47)
		
		gyro_xout_scaled = gyro_xout / 131.0
		gyro_yout_scaled = gyro_yout / 131.0
		gyro_zout_scaled = gyro_zout / 131.0

		print("gyro_xout: ", ("%5d" % gyro_xout), " scaled: ", gyro_xout_scaled)
		print("gyro_yout: ", ("%5d" % gyro_yout), " scaled: ", gyro_yout_scaled)
		print("gyro_zout: ", ("%5d" % gyro_zout), " scaled: ", gyro_zout_scaled)
		 
		print()
		print("Accelerometer")
		print("---------------------")
		 
		acc_xout = read_word_2c(0x3b)
		acc_yout = read_word_2c(0x3d)
		acc_zout = read_word_2c(0x3f)
		 
		acc_xout_scaled = acc_xout / 16384.0
		acc_yout_scaled = acc_yout / 16384.0
		acc_zout_scaled = acc_zout / 16384.0
		 
		print("acc_xout: ", ("%6d" % acc_xout), " skaliert: ", acc_xout_scaled)
		print("acc_yout: ", ("%6d" % acc_yout), " skaliert: ", acc_yout_scaled)
		print("acc_zout: ", ("%6d" % acc_zout), " skaliert: ", acc_zout_scaled)
		 
		print("X Rotation: " , get_x_rotation(acc_xout_scaled, acc_yout_scaled, acc_zout_scaled))
		print("Y Rotation: " , get_y_rotation(acc_xout_scaled, acc_yout_scaled, acc_zout_scaled))

		time.sleep(0.1)


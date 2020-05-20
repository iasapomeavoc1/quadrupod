import robot.lx16a_instructions as servo
import numpy as np

class Actuator_Interface():
	def __init__(self,serial_port,ID_list,calibration):
		self.serial_port = serial_port
		self.ID_list = ID_list
		self.calibration = calibration ## a tuple of two arrays (4,3) of offsets and scaling factors (in counts)

	def get_initial_actuator_state(self):
		return self.counts_to_angs(np.reshape(servo.get_actuator_positions(self.serial_port,self.ID_list),(4,3)))

	def angs_to_counts(self,angs):
		return angs*self.calibration[1]+self.calibration[0]

	def counts_to_angs(self,counts):
		return (counts-self.calibration[0])/self.calibration[1]

import numpy as np
import lx16a_instructions as servo
import kinematics as kine
import simulator as sim

class Brain():
	def __init__(self,initial_actuator_state):
		self.actuator_state = initial_actuator_state
		self.joints,self.endpoint_state = kine.fk(initial_actuator_state)
		self.time_step = 10 #milliseconds
		self.limp = False

		self.idx = 0
		self.traj1 = np.array([[95,90,-100],
							   [95,-90,-100],
							   [-95,90,-100],
							   [-95,-90,-100]])
		self.traj1_len = self.traj1.shape

	def update_sensor_state(self):
		pass

	def update_trajectory(self):
		pass

	def update_endpoint_state(self):
		self.endpoint_state = self.traj1[self.idx-1]
		self.idx+=1

	def update_actuator_state(self):
		self.actuator_state = ik(self.endpoint_state,self.actuator_state)

	def send_actuator_commands(self,actuator_interface):
		## Unravel the actuator state to a single dimension list to be iterated over
		actuator_commands = np.ravel(self.actuator_state)
		if self.limp:
			## Unload all servos
			servo.unload_servos(actuator_interface.serial_port,actuator_interface.ID_list)
		else:
			## Iterate over the actuator ID's and command the actuator states over the serial interface
			for i,ID in enumerate(actuator_interface.ID_list):
				servo.LobotSerialServoMove(actuator_interface.serial_port,ID,actuator_commands[i],self.time_step)

class Actuator_Interface():
	def __init__(self,serial_port,ID_list,calibration):
		self.serial_port = serial_port
		self.ID_list = ID_list
		self.calibration = calibration ## a tuple of two arrays (4,3) of offsets and scaling factors (in counts)

	def get_initial_actuator_state(self):
		return counts_to_angs(np.reshape(servo.get_actuator_positions(self.serial_port,self.ID_list),(4,3)))

	def angs_to_counts(self,angs):
		return angs*self.calibration[1]+self.calibration[0]

	def counts_to_angs(self,counts):
		return (counts-self.calibration[0])/self.calibration[1]


def main():	
	Actuator_Interface = Actuator_Interface(serial.Serial('/dev/ttyS0',115200,timeout=1),
										    (1,2,3,4,5,6,7,8,9,10,11,12),
											(np.array([[521.0,295.0,707.0],
							 						   [484.0,222.0,656.0],
							 						   [495.0,343.0,609.0],
							 						   [483.0,205.0,656.0]]),
											 np.array([[250.0,250.0,240.0],
													   [250.0,240.0,240.0],
													   [240.0,250.0,240.0],
										 			   [240.0,250.0,240.0]])))

	Brain = Brain(Actuator_Interface.get_initial_actuator_state())

	sim_on = True
	if sim_on:
		Simulator = sim.Simulator()
		Simulator.set_endpoint(Brain.endpoint_state)
		Simulator.set_joints(Brain.joints)

	while True:
		Brain.update_sensor_state()
		Brain.update_trajectory()
		Brain.update_endpoint_state()
		Brain.update_actuator_state()
		#Brain.send_actuator_commands()

		if sim_on:
			Simulator.set_endpoint(Brain.endpoint_state)
			Simulator.set_joints(Brain.joints)
			Simulator.update_plot()


if __name__ == '__main__':
	main()
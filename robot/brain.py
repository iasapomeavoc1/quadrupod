import robot.kinematics as kine
import numpy as np
import math
pi = math.pi

class Brain():
	def __init__(self,actuator_interface,sensor_interface):
		self.actuator_interface = actuator_interface #Actuator_Interface class, or None
		self.sensor_interface = sensor_interface #Sensor_Interface class, or None

		if self.actuator_interface is not None:
			self.actuator_state = actuator_interface.get_initial_actuator_state() # On Robot, measure servo angles
		else:
			self.actuator_state = kine.default_actuator_state() # In Simulation, start at default

		self.joints,self.endpoint_state = kine.fk(self.actuator_state)

		self.time_step = 10 #milliseconds

		self.limp = False

		self.idx = 0

		self.traj = self.endpoint_state

		## Trot while moving left
		# cycle_len = 20
		# twu = kine.generate_triangle_wave(-150,-100,cycle_len)
		# twd = kine.generate_triangle_wave(-100,-150,cycle_len)
		# twl = kine.generate_triangle_wave(-10,10,cycle_len)
		# twr = kine.generate_triangle_wave(10,-10,cycle_len)
		# for n in range(10):
		# 	for i in range(cycle_len):
		# 		self.traj = np.append(self.traj,
		# 							  [[[95],[90+twl[i]],[twu[i]]],
		# 							   [[95],[-90+twr[i]],[twd[i]]],
		# 							   [[-95],[90+twr[i]],[twd[i]]],
		# 							   [[-95],[-90+twl[i]],[twu[i]]]],2)

		## Body R-P-Y
		cycle_len = 20
		twr = kine.generate_triangle_wave(-pi/10,pi/10,cycle_len)
		twp = kine.generate_triangle_wave(-pi/10,pi/10,cycle_len)
		twy = kine.generate_triangle_wave(-pi/10,pi/10,cycle_len)
		for n in range(10):
			for i in range(cycle_len):
				self.traj = np.append(self.traj, kine.generate_body_rpy(self.traj[:,:,0],twr[i],twp[i],twy[i]),2)

		self.traj_len = self.traj.shape[2]

	def update_sensor_state(self):
		pass

	def update_trajectory(self):
		pass

	def update_endpoint_state(self):
		self.endpoint_state = self.traj[:,:,self.idx-1]
		self.idx+=1
		if self.idx>self.traj_len:
			self.limp = True

	def update_actuator_state(self):
		self.joints,self.actuator_state = kine.ik(self.endpoint_state,self.actuator_state)

	def send_actuator_commands(self):
		## Unravel the actuator state to a single dimension list to be iterated over
		if self.actuator_interface is not None:
			actuator_commands = np.ravel(self.actuator_state)
			if self.limp:
				## Unload all servos
				servo.unload_servos(self.actuator_interface.serial_port,actuator_interface.ID_list)
			else:
				## Iterate over the actuator ID's and command the actuator states over the serial interface
				for i,ID in enumerate(self.actuator_interface.ID_list):
					servo.LobotSerialServoMove(self.actuator_interface.serial_port,ID,actuator_commands[i],self.time_step)

	def print_diagnostics(self):
		print("Endpoint State: ", self.endpoint_state.ravel())
		print("Actuator State: ", self.actuator_state.ravel())

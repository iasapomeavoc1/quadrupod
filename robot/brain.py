import robot.kinematics as kine
import numpy as np
import math
pi = math.pi

class Brain():
	def __init__(self,actuator_interface,sensor_interface):
		self.actuator_interface = actuator_interface #Actuator_Interface class, or None
		self.sensor_interface = sensor_interface #Sensor_Interface class, or None

		if self.actuator_interface is not None:
			self.actuator_state = actuator_interface.get_actuator_angs() # On Robot, measure servo angles
			print(self.actuator_state)
		else:
			self.actuator_state = kine.default_actuator_state() # In Simulation, start at default

		self.joints,self.endpoint_state = kine.fk(self.actuator_state)

		self.time_step = 10 #milliseconds

		self.limp = False

		self.idx = 0

		self.traj = self.endpoint_state

		## Trot in place
		cycle_len = 24
		twu = kine.generate_triangle_wave(-150,-120,cycle_len)
		twd = kine.generate_triangle_wave(-120,-150,cycle_len)
		for n in range(5):
			for i in range(cycle_len):
				self.traj = np.append(self.traj,
									  [[[95],[90],[twu[i]]],
									   [[95],[-90],[twd[i]]],
									   [[-95],[90],[twd[i]]],
									   [[-95],[-90],[twu[i]]]],2)

		## Trot while moving left
		cycle_len = 24
		twu = kine.generate_triangle_wave(-150,-120,cycle_len)
		twd = kine.generate_triangle_wave(-120,-150,cycle_len)
		twl = kine.generate_triangle_wave(-10,10,cycle_len)
		twr = kine.generate_triangle_wave(10,-10,cycle_len)
		for n in range(5):
			for i in range(cycle_len):
				self.traj = np.append(self.traj,
									  [[[95],[90+twl[i]],[twu[i]]],
									   [[95],[-90+twr[i]],[twd[i]]],
									   [[-95],[90+twr[i]],[twd[i]]],
									   [[-95],[-90+twl[i]],[twu[i]]]],2)

		## Trot while moving right
		cycle_len = 24
		twu = kine.generate_triangle_wave(-150,-120,cycle_len)
		twd = kine.generate_triangle_wave(-120,-150,cycle_len)
		twl = kine.generate_triangle_wave(-10,10,cycle_len)
		twr = kine.generate_triangle_wave(10,-10,cycle_len)
		for n in range(5):
			for i in range(cycle_len):
				self.traj = np.append(self.traj,
									  [[[95],[90+twr[i]],[twu[i]]],
									   [[95],[-90+twl[i]],[twd[i]]],
									   [[-95],[90+twl[i]],[twd[i]]],
									   [[-95],[-90+twr[i]],[twu[i]]]],2)

		## Body R-P-Y
		# cycle_len = 20
		# twr = kine.generate_triangle_wave(-pi/10,pi/10,cycle_len)
		# twp = kine.generate_triangle_wave(-pi/10,pi/10,cycle_len)
		# twy = kine.generate_triangle_wave(-pi/10,pi/10,cycle_len)
		# for n in range(10):
		# 	for i in range(cycle_len):
		# 		self.traj = np.append(self.traj, kine.generate_body_rpy(self.traj[:,:,0],twr[i],twp[i],twy[i]),2)

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
			if self.limp:
				## Unload all servos
				self.actuator_interface.unload_servos()
			else:
				actuator_commands = np.ravel(self.actuator_interface.angs_to_counts(self.actuator_state))
				## Iterate over the actuator ID's and command the actuator states over the serial interface
				## get this iteration into actuators.py
				for i,ID in enumerate(self.actuator_interface.ID_list):
					#print(ID,actuator_commands[i],self.time_step)
					self.actuator_interface.LobotSerialServoMove(ID,int(actuator_commands[i]),self.time_step)

	def print_diagnostics(self):
		print("Voltage: ",self.actuator_interface.LobotSerialServoReadVin(1))
		print("Endpoint State: ", self.endpoint_state.ravel())
		print("Actuator State: ", self.actuator_state.ravel())

import robot.brain as brain
import robot.actuators as actr
import robot.sensors as snsr
import numpy as np
import time
import os

#os.system('sudo chmod 0777 /dev/ttyS0')
#os.system('sudo chown ubuntu:ubuntu /dev/ttyS0')

#port_name = '/dev/ttyS0'
port_name = '/dev/ttyUSB0'
baud_rate = 115200
ID_list = (1,2,3,4,5,6,7,8,9,10,11,12)

calibration = (np.array([[499.0,299.0,725.0],[481.0,229.0,711.0],[502.0,352.0,625.0],[477.0,205.0,707.0]]),
 			   np.array([[246.0,232.0,243.0],[236.0,233.0,240.0],[236.0,238.0,247.0],[241.0,226.0,231.0]]))

actuator_interface = actr.Actuator_Interface(port_name,baud_rate,ID_list,calibration)

#sensor_interface = snsr.Sensor_Interface()
sensor_interface = None
#print(sensor_interface.get_imu_data())

robo_doggo_brain = brain.Brain(actuator_interface,sensor_interface)
i=0
while not robo_doggo_brain.limp:
	try:
		i+=1
		print("i = ",i)
		robo_doggo_brain.update_sensor_state()
		robo_doggo_brain.update_trajectory()
		robo_doggo_brain.update_endpoint_state()
		robo_doggo_brain.update_actuator_state()
		robo_doggo_brain.send_actuator_commands()
		#robo_doggo_brain.print_diagnostics()
		time.sleep(robo_doggo_brain.time_step/1000.0)
	except KeyboardInterrupt:
		robo_doggo_brain.limp = True
		robo_doggo_brain.send_actuator_commands()		




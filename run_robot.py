import robot.brain as brain
import robot.actuators as actr
import robot.sensors as snsr
import serial
import numpy as np

servo_ID_list = (1,2,3,4,5,6,7,8,9,10,11,12)
serial_port = serial.Serial('/dev/ttyS0',115200,timeout=1)
calibration = (np.array([[521.0,295.0,707.0],[484.0,222.0,656.0],[495.0,343.0,609.0],[483.0,205.0,656.0]]),
			   np.array([[250.0,250.0,240.0],[250.0,240.0,240.0],[240.0,250.0,240.0],[240.0,250.0,240.0]]))

actuator_interface = actr.Actuator_Interface(serial_port,servo_ID_list,calibration)
sensor_interface = snsr.Sensor_Interface()

print(sensor_interface.get_imu_data())

robo_doggo_brain = brain.Brain(actuator_interface,sensor_interface)

while not robo_doggo_brain.limp:
	robo_doggo_brain.update_sensor_state()
	robo_doggo_brain.update_trajectory()
	robo_doggo_brain.update_endpoint_state()
	robo_doggo_brain.update_actuator_state()
	robo_doggo_brain.print_diagnostics()

	time.sleep(robo_doggo_brain.time_step/1000.0)

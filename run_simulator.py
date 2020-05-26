import robot.brain as brain
import simulation.simulator as sim
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')
import time

actuator_interface = None
sensor_interface = None

robo_doggo_brain = brain.Brain(actuator_interface,sensor_interface)

fig = plt.figure()
robo_doggo_simulator = sim.Simulator(fig,[robo_doggo_brain.joints,robo_doggo_brain.endpoint_state])
#robo_doggo_simulator.update_plot()

while not robo_doggo_brain.limp:
	robo_doggo_brain.update_endpoint_state()
	robo_doggo_brain.update_actuator_state()
	#robo_doggo_brain.print_diagnostics()

	robo_doggo_simulator.set_joints(robo_doggo_brain.joints)
	robo_doggo_simulator.set_endpoint(robo_doggo_brain.endpoint_state)
	robo_doggo_simulator.update_plot()

	time.sleep(robo_doggo_brain.time_step/1000.0)

print("Ctrl-C to Stop")
plt.show()

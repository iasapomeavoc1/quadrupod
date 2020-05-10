import matplotlib
matplotlib.use('TkAgg')
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

class Simulator():
	def __init__(self, figure):
		self.fig = figure
		self.ax = Axes3D(self.fig)
		ax.scatter(0,0,0,c='r',marker='x')
		ax.set_xlabel('X')
		ax.set_ylabel('Y')
		ax.set_zlabel('Z')
		Min = -200
		Max = 200
		ax.scatter([Min,Min,Min,Min,Max,Max,Max,Max],[Min,Min,Max,Max,Min,Min,Max,Max],[Min,Max,Max,Min,Min,Max,Max,Min],c='w')

		self.actuator_state = np.zeros((4,3))
		self.endpoint_state = np.zeros((4,3))
		
		self.plt_leg_a = ax.scatter(self.joints[0:3,0],self.joints[0:3,1],self.joints[0:3,2],c='r',marker='.',s=50)
		self.plt_leg_b = ax.scatter(self.joints[3:6,0],self.joints[3:6,1],self.joints[3:6,2],c='g',marker='.',s=50)
		self.plt_leg_c = ax.scatter(self.joints[6:9,0],self.joints[6:9,1],self.joints[6:9,2],c='b',marker='.',s=50)
		self.plt_leg_d = ax.scatter(self.joints[9:12,0],self.joints[9:12,1],self.joints[9:12,2],c='c',marker='.',s=50)
		self.plt_endpoint_state = ax.scatter(self.endpoint_state[:,0],self.endpoint_state[:,1],self.endpoint_state[:,2],c='r',marker='x')

		self.fig.canvas.draw()

	def set_joints(self,joints):
		self.joints = joints

	def set_endpoint(self,endpoint_state):
		self.endpoint_state = endpoint_state

	def update_plot(self):
		Anewdata = (tuple(map(tuple,self.joints[0:3,0].reshape([1,3])))[0],tuple(map(tuple,self.joints[0:3,1].reshape([1,3])))[0],self.joints[0:3,2].reshape([1,3])[0])
		Bnewdata = (tuple(map(tuple,self.joints[3:6,0].reshape([1,3])))[0],tuple(map(tuple,self.joints[3:6,1].reshape([1,3])))[0],self.joints[3:6,2].reshape([1,3])[0])
		Cnewdata = (tuple(map(tuple,self.joints[6:9,0].reshape([1,3])))[0],tuple(map(tuple,self.joints[6:9,1].reshape([1,3])))[0],self.joints[6:9,2].reshape([1,3])[0])
		Dnewdata = (tuple(map(tuple,self.joints[9:12,0].reshape([1,3])))[0],tuple(map(tuple,self.joints[9:12,1].reshape([1,3])))[0],self.joints[9:12,2].reshape([1,3])[0])
		newTarget = (tuple(self.endpoint_state[:,0]),tuple(self.endpoint_state[:,1]),self.endpoint_state[:,2])
		self.plt_leg_a._offsets3d = Anewdata
		self.plt_leg_b._offsets3d = Bnewdata
		self.plt_leg_c._offsets3d = Cnewdata
		self.plt_leg_d._offsets3d = Dnewdata
		self.plt_endpoint_state._offsets3d = newTarget
		self.fig.canvas.draw()

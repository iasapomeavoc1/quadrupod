from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

class Simulator():
	def __init__(self, figure, initial_state):
		self.fig = figure
		self.ax = Axes3D(self.fig)

		self.ax.scatter(0,0,0,c='r',marker='x')
		self.ax.set_xlabel('X')
		self.ax.set_ylabel('Y')
		self.ax.set_zlabel('Z')
		Min = -200
		Max = 200
		self.ax.scatter([Min,Min,Min,Min,Max,Max,Max,Max],[Min,Min,Max,Max,Min,Min,Max,Max],[Min,Max,Max,Min,Min,Max,Max,Min],c='w')

		self.joints = initial_state[0]
		self.endpoint_state = initial_state[1]

		self.body_pts = np.array([[95,90,0],[95,-90,0],[-95,-90,0],[-95,90,0],[95,90,0]])
		self.plt_body = self.ax.plot(self.body_pts[:,0],self.body_pts[:,1],self.body_pts[:,2],c='k',marker='o')
		self.plt_leg_a, = self.ax.plot(self.joints[0:3,0].ravel(),self.joints[0:3,1].ravel(),self.joints[0:3,2].ravel(),c='r',marker='o')
		self.plt_leg_b, = self.ax.plot(self.joints[3:6,0].ravel(),self.joints[3:6,1].ravel(),self.joints[3:6,2].ravel(),c='g',marker='o')
		self.plt_leg_c, = self.ax.plot(self.joints[6:9,0].ravel(),self.joints[6:9,1].ravel(),self.joints[6:9,2].ravel(),c='b',marker='o')
		self.plt_leg_d, = self.ax.plot(self.joints[9:12,0].ravel(),self.joints[9:12,1].ravel(),self.joints[9:12,2].ravel(),c='c',marker='o')
		self.plt_endpoint_state = self.ax.scatter(self.endpoint_state[:,0].ravel(),self.endpoint_state[:,1].ravel(),self.endpoint_state[:,2].ravel(),c='k',marker='x')

	def set_joints(self,joints):
		self.joints = joints

	def set_endpoint(self,endpoint_state):
		self.endpoint_state = endpoint_state

	def update_plot(self):
		Anewdata = (tuple(map(tuple,self.joints[0:3,0].reshape([1,3])))[0],tuple(map(tuple,self.joints[0:3,1].reshape([1,3])))[0],self.joints[0:3,2].reshape([1,3])[0])
		Bnewdata = (tuple(map(tuple,self.joints[3:6,0].reshape([1,3])))[0],tuple(map(tuple,self.joints[3:6,1].reshape([1,3])))[0],self.joints[3:6,2].reshape([1,3])[0])
		Cnewdata = (tuple(map(tuple,self.joints[6:9,0].reshape([1,3])))[0],tuple(map(tuple,self.joints[6:9,1].reshape([1,3])))[0],self.joints[6:9,2].reshape([1,3])[0])
		Dnewdata = (tuple(map(tuple,self.joints[9:12,0].reshape([1,3])))[0],tuple(map(tuple,self.joints[9:12,1].reshape([1,3])))[0],self.joints[9:12,2].reshape([1,3])[0])
		newTarget = (tuple(self.endpoint_state[:,0].ravel()),tuple(self.endpoint_state[:,1].ravel()),self.endpoint_state[:,2].ravel())
		
		self.plt_leg_a.set_data(Anewdata[0],Anewdata[1])
		self.plt_leg_a.set_3d_properties(Anewdata[2])
		self.plt_leg_b.set_data(Bnewdata[0],Bnewdata[1])
		self.plt_leg_b.set_3d_properties(Bnewdata[2])
		self.plt_leg_c.set_data(Cnewdata[0],Cnewdata[1])
		self.plt_leg_c.set_3d_properties(Cnewdata[2])
		self.plt_leg_d.set_data(Dnewdata[0],Dnewdata[1])
		self.plt_leg_d.set_3d_properties(Dnewdata[2])
		self.plt_endpoint_state._offsets3d = newTarget

		plt.show(block=False)
		plt.pause(0.0001)

import numpy as np
import math
pi = math.pi

## Robot Dimensions
O1A = np.array([[95],[90],[18]]); O1B = np.array([[95],[-90],[18]]); O1C = np.array([[-95],[90],[18]]); O1D = np.array([[-95],[-90],[18]]);
O2A = np.array([[0],[25],[0]]);   O2B = np.array([[0],[25],[0]]);    O2C = np.array([[0],[-25],[0]]);    O2D = np.array([[0],[-25],[0]]);
O3A = np.array([[0],[80],[0]]);   O3B = np.array([[0],[80],[0]]);    O3C = np.array([[0],[80],[0]]);    O3D = np.array([[0],[80],[0]]);
O4A = np.array([[0],[-95],[0]]);  O4B = np.array([[0],[-95],[0]]);   O4C = np.array([[0],[-95],[0]]);   O4D = np.array([[0],[-95],[0]]);

def generate_body_rpy(pts,r,p,y):
	rc = math.cos(r); rs = math.sin(r)
	pc = math.cos(p); ps = math.sin(p)
	yc = math.cos(y); ys = math.sin(y)

	return np.array([[[pts[0,0]],[pts[0,1]*rc-pts[0,2]*rs],[pts[0,1]*rs+pts[0,2]*rc]],
					 [[pts[1,0]],[pts[1,1]*rc-pts[1,2]*rs],[pts[1,1]*rs+pts[1,2]*rc]],
					 [[pts[2,0]],[pts[2,1]*rc-pts[2,2]*rs],[pts[2,1]*rs+pts[2,2]*rc]],
					 [[pts[3,0]],[pts[3,1]*rc-pts[3,2]*rs],[pts[3,1]*rs+pts[3,2]*rc]]])

def generate_triangle_wave(bottom,top,frequency):
	wave = np.array([])
	wave = np.append(wave,np.linspace(bottom,top,int(frequency/2)))
	wave = np.append(wave,np.linspace(top,bottom,int(frequency/2)))
	return wave

def default_actuator_state():
	return np.array([[0.00,2.3,-.3],[0.00,2.3,-.3],
					 [0.00,2.3,-.3],[0.00,2.3,-.3]])

def calcJacobian(pos,angs,joints):
	JA = np.array([[0,-O3A[1]*math.sin(angs[0,1]),O4A[1]*math.sin(angs[0,2])],
				   [math.cos(angs[0,0])*(O2A[1]+O3A[1]*math.sin(angs[0,1])+O4A[1]*math.sin(angs[0,2])), math.cos(angs[0,1])*O3A[1]*math.sin(angs[0,0]),  math.cos(angs[0,2])*O4A[1]*math.sin(angs[0,0])],
				   [math.sin(angs[0,0])*(O2A[1]+O3A[1]*math.sin(angs[0,1])+O4A[1]*math.sin(angs[0,2])), -math.cos(angs[0,1])*O3A[1]*math.cos(angs[0,0]),  -math.cos(angs[0,2])*O4A[1]*math.cos(angs[0,0])]],dtype='float')
	JAt = np.linalg.inv(JA)

	JB = np.array([[0,-O3B[1]*math.sin(angs[1,1]),O4B[1]*math.sin(angs[1,2])],
				   [math.cos(angs[1,0])*(O2B[1]+O3B[1]*math.sin(angs[1,1])+O4B[1]*math.sin(angs[1,2])), math.cos(angs[1,1])*O3B[1]*math.sin(angs[1,0]),  math.cos(angs[1,2])*O4B[1]*math.sin(angs[1,0])],
				   [math.sin(angs[1,0])*(O2B[1]+O3B[1]*math.sin(angs[1,1])+O4B[1]*math.sin(angs[1,2])), -math.cos(angs[1,1])*O3B[1]*math.cos(angs[1,0]),  -math.cos(angs[1,2])*O4B[1]*math.cos(angs[1,0])]],dtype='float')
	JBt = np.linalg.inv(JB)

	JC = np.array([[0,O3C[1]*math.sin(angs[2,1]),-O4C[1]*math.sin(angs[2,2])],
				   [(math.cos(angs[2,0])*(O2C[1]-O3C[1]*math.sin(angs[2,1])-O4C[1]*math.sin(angs[2,2]))), -math.cos(angs[2,1])*O3C[1]*math.sin(angs[2,0]),  -math.cos(angs[2,2])*O4C[1]*math.sin(angs[2,0])],
				   [-(math.sin(angs[2,0])*(O2C[1]-O3C[1]*math.sin(angs[2,1])-O4C[1]*math.sin(angs[2,2]))),-math.cos(angs[2,1])*O3C[1]*math.cos(angs[2,0]),  -math.cos(angs[2,2])*O4C[1]*math.cos(angs[2,0])]],dtype='float')
	JCt = np.linalg.inv(JC)

	JD = np.array([[0,O3D[1]*math.sin(angs[3,1]),-O4D[1]*math.sin(angs[3,2])],
				   [(math.cos(angs[3,0])*(O2D[1]-O3D[1]*math.sin(angs[3,1])-O4D[1]*math.sin(angs[3,2]))), -math.cos(angs[3,1])*O3D[1]*math.sin(angs[3,0]),  -math.cos(angs[3,2])*O4D[1]*math.sin(angs[3,0])],
				   [-(math.sin(angs[3,0])*(O2D[1]-O3D[1]*math.sin(angs[3,1])-O4D[1]*math.sin(angs[3,2]))),-math.cos(angs[3,1])*O3D[1]*math.cos(angs[3,0]),  -math.cos(angs[3,2])*O4D[1]*math.cos(angs[3,0])]],dtype='float')
	JDt = np.linalg.inv(JD)

	errA = pos[0,:].reshape([3,1]) - joints[2,:]
	errB = pos[1,:].reshape([3,1]) - joints[5,:]
	errC = pos[2,:].reshape([3,1]) - joints[8,:]
	errD = pos[3,:].reshape([3,1]) - joints[11,:]

	angs = angs + np.squeeze(np.array([np.dot(JAt,errA),np.dot(JBt,errB),np.dot(JCt,errC),np.dot(JDt,errD)]))

	return angs,np.linalg.norm(errA),np.linalg.norm(errB),np.linalg.norm(errC),np.linalg.norm(errD)

def modPi(angs):
	redangs = angs
	for i in range(len(angs)):
		for j in range(len(angs[i])):
			a = angs[i,j]%(2*pi)
			if a>pi:
				a = a-2*pi
			elif a<-pi:
				a = -2*pi-a
			redangs[i,j]=a
	return redangs

def ik(pos,seed): 
	# Input: end effector position command (4x3) [mm] and seed angles (4x3) [radians]
	# Output: joint angles (4x3) [radians], number of iterations it took [int]
	angs = seed
	joints,endpoints = fk(angs)
	angs,errA,errB,errC,errD = calcJacobian(pos,angs,joints)
	iterations = 0
	while errA>1 or errB>1 or errC>1 or errD>1:
		angs = modPi(angs)
		iterations+=1
		joints,endpoints = fk(angs)
		angs,errA,errB,errC,errD = calcJacobian(pos,angs,joints)
		if iterations>300:
			angs = seed
			print("TOOK TOO LONG TO CONVERGE, ABORT")
			#print(errA,errB,errC,errD)
			break
	print("IK DONE in ",iterations," iterations")
	return joints,angs

def fk(angs):
	# Input: joint angles (3x4) [radians]
	# Output: all joint origins (3x12) [mm] (end effectors are at index 2,5,8,11)

	U1A = np.array([[0,                     0,                   1],
					 [-math.cos(angs[0,0]), math.sin(angs[0,0]), 0],
					 [-math.sin(angs[0,0]), -math.cos(angs[0,0]),0]])
	U2A = np.array([[0,                     0,                  -1],
					 [-math.cos(angs[0,1]), math.sin(angs[0,1]), 0],
					 [math.sin(angs[0,1]),  math.cos(angs[0,1]), 0]])	
	U3A = np.array([[0,                     0,                   1],
					 [-math.cos(angs[0,2]), math.sin(angs[0,2]), 0],
					 [-math.sin(angs[0,2]), -math.cos(angs[0,2]),0]])	
	U1B = np.array([[0,                     0,                   1],
					 [-math.cos(angs[1,0]), math.sin(angs[1,0]), 0],
					 [-math.sin(angs[1,0]), -math.cos(angs[1,0]),0]])
	U2B = np.array([[0,                     0,                  -1],
					 [-math.cos(angs[1,1]), math.sin(angs[1,1]), 0],
					 [math.sin(angs[1,1]),  math.cos(angs[1,1]), 0]])	
	U3B = np.array([[0,                     0,                   1],
					 [-math.cos(angs[1,2]), math.sin(angs[1,2]), 0],
					 [-math.sin(angs[1,2]), -math.cos(angs[1,2]),0]])	
	U1C = np.array([[0,                     0,                  -1],
					 [-math.cos(angs[2,0]), math.sin(angs[2,0]),0],
					 [math.sin(angs[2,0]), math.cos(angs[2,0]), 0]])
	U2C = np.array([[0,                     0,                   1],
					 [math.cos(angs[2,1]), -math.sin(angs[2,1]), 0],
					 [math.sin(angs[2,1]), math.cos(angs[2,1]), 0]])	
	U3C = np.array([[0,                     0,                   -1],
					 [math.cos(angs[2,2]), -math.sin(angs[2,2]), 0],
					 [-math.sin(angs[2,2]), -math.cos(angs[2,2]),0]])	
	U1D = np.array([[0,                     0,                  -1],
					 [-math.cos(angs[3,0]), math.sin(angs[3,0]),0],
					 [math.sin(angs[3,0]), math.cos(angs[3,0]), 0]])
	U2D = np.array([[0,                     0,                   1],
					 [math.cos(angs[3,1]), -math.sin(angs[3,1]), 0],
					 [math.sin(angs[3,1]), math.cos(angs[3,1]), 0]])	
	U3D = np.array([[0,                     0,                   -1],
					 [math.cos(angs[3,2]), -math.sin(angs[3,2]), 0],
					 [-math.sin(angs[3,2]), -math.cos(angs[3,2]),0]])	

	joints = np.array([O1A + np.dot(U1A,O2A) , O1A + np.dot(U1A,(O2A+np.dot(U2A,O3A))) ,O1A + np.dot(U1A,(O2A + np.dot(U2A,O3A) + np.dot(U3A,O4A))),
					   O1B + np.dot(U1B,O2B) , O1B + np.dot(U1B,(O2B+np.dot(U2B,O3B))) ,O1B + np.dot(U1B,(O2B + np.dot(U2B,O3B) + np.dot(U3B,O4B))),
					   O1C + np.dot(U1C,O2C) , O1C + np.dot(U1C,(O2C+np.dot(U2C,O3C))) ,O1C + np.dot(U1C,(O2C + np.dot(U2C,O3C) + np.dot(U3C,O4C))),
					   O1D + np.dot(U1D,O2D) , O1D + np.dot(U1D,(O2D+np.dot(U2D,O3D))) ,O1D + np.dot(U1D,(O2D + np.dot(U2D,O3D) + np.dot(U3D,O4D)))])

	endpoints = joints[(2,5,8,11),:]

	return joints, endpoints
import numpy as np

unit = np.pi/180

def RX(t):
	return np.array([[1, 0, 0], 
					[0, np.cos(t), -np.sin(t)], 
					[0, np.sin(t), np.cos(t)]])

def RY(t):
	return np.array([[np.cos(t), 0, np.sin(t)],
					 [0, 1, 0], 
					 [-np.sin(t), 0, np.cos(t)]])
def RZ(t):
	return np.array([[np.cos(t), -np.sin(t), 0],
					 [np.sin(t), np.cos(t), 0], 
					 [0, 0, 1]])

def simplecheck(rot):
	print(np.linalg.norm(rot[:,0]))
	print(np.linalg.norm(rot[:,1]))
	print(np.linalg.norm(rot[:,2]))

def Tr(R,P):
	'''
		R: Rotation Matrix
		P: Translation
	'''
	T = np.zeros([4,4])
	T[3,3] = 1
	T[0:3,0:3] = R
	T[0:3,3] = P
	return T

def smul(lst):
	dim = lst[0].shape[0]
	T = np.eye(dim,dim)
	for matrix in lst:
		T = T.dot(matrix)
	return T

def RPY(C,B,A):
	'''
		1. Rotate about x: C (Row)
		2. Rotate about y: B (Pitch)
		3. Rotate about z: A (Yaw)
	'''
	return smul([RZ(A),RY(B),RX(C)])

def Euler_Angles(A,B,C):
	'''
		1. Rotate about z: A 
		2. Rotate about y: B 
		3. Rotate about x: C 
	'''
	return smul([RZ(A),RY(B),RX(C)])

def quat_from_aa(K,t):
	'''
		K = [Kx, Ky, Kz] numpy array
		t = theta 
	'''
	kx, ky, kz = K[0], K[1], K[2]
	norm = np.sqrt(kx*kx+ky*ky+kz*kz)
	w = np.cos(t/2)
	x = kx*np.sin(t/2)/norm
	y = ky*np.sin(t/2)/norm
	z = kz*np.sin(t/2)/norm
	return np.array([w,x,y,z])


def quat2rot(q):
	'''
		transform quat to rotation
	'''
	w, x, y ,z = q[0], q[1], q[2], q[3]
	rot = np.zeros([3,3])

	rot[0,0] = 1 - 2*y**2 - 2*z**2
	rot[0,1] = 2*(x*y-w*z)
	rot[0,2] = 2*(w*y+x*z)

	rot[1,0] = 2*(w*z+y*x)
	rot[1,1] = 1 - 2*x**2 - 2*z**2
	rot[1,2] = 2*(y*z-x*w)

	rot[2,0] = 2*(x*z-w*y)
	rot[2,1] = 2*(y*z+x*w)
	rot[2,2] = 1 - 2*x**2 - 2*y**2

	return rot


def qtimes(p,q):
	'''	
		quat mutiplicaiton:
		q1, q2 are both np.array 4-dim
		reference:
		http://graphics.stanford.edu/courses/cs348a-17-winter/Papers/quaternion.pdf
	'''
	p0, p = p[0], p[1:4]
	q0, q = q[0], q[1:4]

	w = p0*q0 - np.inner(p,q)
	v = p0*q + q0*p + np.cross(p,q)
	result = np.zeros([4,])
	result[0] = w
	result[1:4] = v
	return result






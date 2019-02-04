import numpy as np
from lib.rotation import *

# check quat_from_aa & quat2rot
# t = 45*unit
# q1 = np.array([np.cos(t/2), 0, np.sin(t/2), 0])
# print(quat2rot(q1))

# q2 = quat_from_aa(np.array([0,1,0]), t)
# print(quat2rot(q2))

# check qtimes:
# q1 = np.array([np.cos(15*unit), 0.371*np.sin(15*unit),
# 				0.557*np.sin(15*unit), 0.743*np.sin(15*unit)])
# q2 = np.array([np.cos(22.5*unit), 0.684*np.sin(22.5*unit),
# 				0.570*np.sin(22.5*unit), 0.456*np.sin(22.5*unit)])
# print(qtimes(q1,q2))

# icp2 2.1
R1 = RZ(42*unit)
q2 = quat_from_aa(np.array([3.2, -2.7, 4.4]), 135*unit)
R2 = quat2rot(q2)
R3 = RX(1.6*unit)
result = smul([R1,R2,R3])
simplecheck(result)
print(result)

# icp2 2.4
T1 = Tr(np.eye(3,3),[4.2,0,0])
T2 = Tr(RZ(42*unit),[0,0,0])

q3 = quat_from_aa(np.array([3.2, -2.7, 4.4]), 135*unit)
R3 = quat2rot(q3)
T3 = Tr(R3,[0,0,0])

v4 = np.array([-4,14.7,0.27])
t4 = -16.2*v4/np.linalg.norm(v4)
T4 = Tr(np.eye(3,3),t4)

T5 = Tr(RX(1.6*unit),[0,0,0])
result = smul([T1,T2,T3,T4,T5])
print(result)




import numpy as np
from lib.rotation import *

# hw1 1.3
R1 = RZ(119*unit)
R2 = RY(45*unit)
R3 = RZ(16*unit)
print(smul([R1,R2,R3]))

# hw1 1.4
R1 = RZ(-37.5*unit)
R2 = RX(27.5*unit)
R3 = RZ(126*unit)
print(smul([R1,R2,R3]))

# HW1 1.6 
R1 = RX(20*unit)
R1 = np.dot(R1, RZ(4*unit))
R1 = np.dot(R1, RZ(4*unit))
R1 = np.dot(R1, RX(16*unit))   
R1 = np.dot(R1, RY(45*unit))
print(R1)
# print(smul([RX(20*unit),RZ(4*unit),RZ(4*unit),RX(16*unit),RY(45*unit)]))

# hw1 2.2 car
T1 = Tr(RZ(20*unit),[-10,0,0])
T2 = Tr(RZ(-90*unit),[20,0,0])
T3 = Tr(RY(10*unit),[0,0,0])
T4 = Tr(np.eye(3,3),[10,0,0])

T = smul([T1,T2,T3,T4])
print(T)

# [[ 0.33682409  0.93969262  0.05939117 12.1620933 ]
#  [-0.92541658  0.34202014 -0.16317591 -2.41376292]
#  [-0.17364818  0.          0.98480775 -1.73648178]
#  [ 0.          0.          0.          1.        ]]


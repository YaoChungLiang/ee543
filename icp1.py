import numpy as np
from lib.rotation import *

# ICP1
theta = np.pi/6
RAB = np.dot(RX(theta), RY(2*theta))
print(RX(theta))
print(RY(2*theta))
print(RAB)

# ICP1
R02 = np.dot(RZ(theta), RY(2*theta/3))
P01 = np.array([5,5,0]).T
P02 = np.array([7,1,0]).T

T02 = np.dot(R02.T, (P01-P02))

print(T02)
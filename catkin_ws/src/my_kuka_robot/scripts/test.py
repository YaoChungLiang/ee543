
import os
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
import copy
#from sympy import mpmath
import mpmath
from sympy import *
al0, al1, al2, al3, al4, al5, al6 = symbols('alpha0:7')  	# link_twist
a0, a1, a2, a3, a4, a5, a6 	  = symbols('a0:7')  		# link_length
th1, th2, th3, th4, th5, th6, th7 = symbols('theta1:8')  	# joint_twist
d1, d2, d3, d4, d5, d6, d7 	  = symbols('d1:8')  		# joint_length
s = {al0: 0,     a0: 0,      d1: 0.75,    
     al1: -pi/2, a1: 0.35,   d2: 0,      th2: th2-pi/2,
     al2: 0,     a2: 1.25,   d3: 0,      
     al3: -pi/2, a3: -0.054, d4: 1.5,    
     al4: pi/2,  a4: 0,      d5: 0,      
     al5: -pi/2, a5: 0,      d6: 0,      
     al6: 0,     a6: 0,      d7: 0.303,  th7: 0}

def createMatrix(al, a, th, d):
    # TODO: Fill in the 2-4 rows of matrix
    mat =  Matrix([[         cos(th),        -sin(th),        0,         a],
                   [ sin(th)*cos(al), cos(th)*cos(al), -sin(al),-sin(al)*d],
                   [ sin(th)*sin(al), cos(th)*sin(al),  cos(al), cos(al)*d],
                   [               0,               0,        0,         1]])

    return mat

T0_1 = createMatrix(al0,a0, th1, d1)
T0_1 = T0_1.subs(s)
# TODO: Do the same for the remaining parts
T1_2 = createMatrix(al1,a1, th2, d2).subs(s)
T0_2 = simplify(np.dot(T0_1, T1_2)) # base_link to link 2
theta1 = pi/2
X2_prime = T0_2.subs({th1:theta1, th2:0}).dot([[1], [0], [0], [0]])
print(X2_prime)
pO2_0 = Matrix([[s[a1]], [0], [s[d1]]])
t = copy.deepcopy(pO2_0)
t[0,0] = 100
print(t)
print("insert:"+ str(pO2_0.row_insert(3,Matrix([[0]]))))
print("pO2_0L:"+str(pO2_0))
#print(a1.subs(s))
print(a1.evalf(subs=s))
#print( pO2_0.normalized() )


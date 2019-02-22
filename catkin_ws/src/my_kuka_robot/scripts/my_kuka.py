#!/usr/bin/env python

# import modules
import os
import tf
import sys
import time
import rospy
import numpy as np
import matplotlib.pyplot as plt
import copy

from my_kuka_robot.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sympy import mpmath
from sympy import *

# display debug messages
def debug_log(log):
    if (debug_mode):
        print(log)
    pass

# DH parameters (symbolic form)
al0, al1, al2, al3, al4, al5, al6 = symbols('alpha0:7')  	# link_twist
a0, a1, a2, a3, a4, a5, a6 	  = symbols('a0:7')  		# link_length
th1, th2, th3, th4, th5, th6, th7 = symbols('theta1:8')  	# joint_twist
d1, d2, d3, d4, d5, d6, d7 	  = symbols('d1:8')  		# joint_length

# TODO: fill in the DH table for the kuka arm
# how does this work?
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

# Rotation Matrix about X
def rot_x(q):
    R_x = Matrix([[       1,       0,       0 ],
                  [       0,  cos(q), -sin(q) ],
                  [       0,  sin(q),  cos(q) ]])
    return R_x

# Rotation Matrix about Y
def rot_y(q):
    R_y = Matrix([[  cos(q),       0,  sin(q) ],
                  [       0,       1,       0 ],
                  [ -sin(q),       0,  cos(q) ]])
    return R_y

# Rotation Matrix about Z
def rot_z(q):
    R_z = Matrix([[  cos(q), -sin(q),       0 ],
                  [  sin(q),  cos(q),       0 ],
                  [       0,       0,       1 ]])
    return R_z

def rtod(q):
    return q*180.0/np.pi

# Degree to Radians

def dtor(q):
    return q*np.pi/180.0

def checkInRange(val, low, high, name):
    if((val >= low) & (val <= high)):
        return True
    else:
        print ("        Out of range {}[{}], ({}, {})".format(name, val, low, high))
        return False;

class MyKukaSolver(object):


    def __init__(self):
        tmp = zeros(4,4)
        tmp[0:3,0:3] = simplify(rot_z(pi)*rot_y(-pi/2))
        tmp[3,3] = 1
        # print(tmp)
        self.T0_1 = createMatrix(al0,a0, th1, d1)
        self.T0_1 = self.T0_1.subs(s)
        # TODO: Do the same for the remaining parts
        self.T1_2 = createMatrix(al1,a1, th2, d2).subs(s)
        self.T2_3 = createMatrix(al2,a2, th3, d3).subs(s)
        self.T3_4 = createMatrix(al3,a3, th4, d4).subs(s)
        self.T4_5 = createMatrix(al4,a4, th5, d5).subs(s)
        self.T5_6 = createMatrix(al5,a5, th6, d6).subs(s)
        self.T6_G = createMatrix(al6,a6, th7, d7).subs(s)

        # TODO: Do the same for the remaining parts
        self.T0_2 = simplify(self.T0_1 * self.T1_2) # base_link to link 2
        self.T0_3 = simplify(self.T0_2 * self.T2_3) # base_link to link 3
        self.T0_4 = simplify(self.T0_3 * self.T3_4) 
        self.T0_5 = simplify(self.T0_4 * self.T4_5) 
        self.T0_6 = simplify(self.T0_5 * self.T5_6) 
        self.T0_G = simplify(self.T0_6 * self.T6_G) 

        # TODO: Correction Needed to account for orientation difference between definition
        # of gripper_link in URDF versus DH Convention
        # self.R_corr = Matrix(simplify(something.....))
        self.R_corr = Matrix(simplify(rot_y(pi/2)*rot_z(pi)))

        # Compute complete transform for End effector
        R_corr2 = self.R_corr.row_insert(3, Matrix([[0, 0, 0]]))
        R_corr2 = R_corr2.col_insert(3, Matrix([0, 0, 0, 1]))
        self.T_total = simplify(self.T0_G * R_corr2)
        debug_log("debug R_corr2")        

        # Rotation transform between link 3 and 6, defined symbollically based 
        # on the Modified DH parameters. 
        self.R3_6_prime = (self.T3_4 * self.T4_5 * self.T5_6)
        debug_log("debug 36_1")        
        self.R3_6_prime = self.R3_6_prime[:3,:3]
        debug_log("debug 36_2")        
        # Memoization for theta4 and theta6
        self.old_th4 = 0
        self.old_th6 = 0
        # pass

    def performFK(self, theta_t):
        theta_s = {th1: theta_t[0], th2: theta_t[1], th3: theta_t[2], th4: theta_t[3], th5: theta_t[4], th6:theta_t[5]}
        # theta_s = {q1: theta_t[0], q}
        origin = Matrix([[0], [0], [0], [1]]) 
        # this is each frames' origin

        T0_2_prime = self.T0_2.evalf(subs=theta_s) 
        p2 = T0_2_prime*origin
        rpy2 = tf.transformations.euler_from_matrix(T0_2_prime.tolist())
        quat2 = tf.transformations.quaternion_from_matrix(T0_2_prime.tolist())
        debug_log("        Link 2 position : {}".format(p2.tolist()))

        T0_3_prime = self.T0_3.evalf(subs=theta_s) 
        p3 = T0_3_prime*origin
        rpy3 = tf.transformations.euler_from_matrix(T0_3_prime.tolist())
        quat3 = tf.transformations.quaternion_from_matrix(T0_3_prime.tolist())
        debug_log("        Link 3 position : {}".format(p3.tolist()))

        T0_5_prime = self.T0_5.evalf(subs=theta_s) 
        p5 = T0_5_prime*origin
        rpy5 = tf.transformations.euler_from_matrix(T0_5_prime.tolist())
        quat5 = tf.transformations.quaternion_from_matrix(T0_5_prime.tolist())
        debug_log("        Link 5/Wrist Center position : {}".format(p5.tolist()))

        T0_G_prime = self.T0_G.evalf(subs=theta_s) 
        pG = T0_G_prime*origin
        rpyG = tf.transformations.euler_from_matrix(T0_G_prime.tolist())
        quatG = tf.transformations.quaternion_from_matrix(T0_G_prime.tolist())
        debug_log("        Gripper/End Effector position : {}".format(pG.tolist()))

        # TODO: Find the FK result given theta_s values
        T_total_prime = self.T_total.evalf(subs=theta_s)
        pFinal = T_total_prime*origin
        rpyFinal = tf.transformations.euler_from_matrix(T_total_prime.tolist())
        quatFinal = tf.transformations.quaternion_from_matrix(T_total_prime.tolist())
        debug_log("    EE URDF position:    {}".format(pFinal.tolist()))
        debug_log("    EE URDF orientation: {}".format(quatFinal))

        '''
            how does this work? how can you compute the euler & quaternion 
            from Transportation matrix
        '''
        result = Pose()
        result.position.x = pFinal[0]
        result.position.y = pFinal[1]
        result.position.z = pFinal[2]
        result.orientation.x = quatFinal[0]
        result.orientation.y = quatFinal[1]
        result.orientation.z = quatFinal[2]
        result.orientation.w = quatFinal[3]
        return result

    def performIK(self, pose):

        def ComputeTriangle(A,B,C):
            return (B**2+C**2-A**2)/(2*B*C)        
        
        def solveTriangle(A,B,C):
            # print(np.arccos(0.5))
            c1 = ComputeTriangle(A,B,C)
            c2 = ComputeTriangle(B,A,C)
            c3 = ComputeTriangle(C,A,B)
            # print("c1,c2,c3:",c1,c2,c3)
            # print("c1 type:", type(c1))
            # print("c2 type:", type(c2))
            # print("c3 type:", type(c3))
            alpha = np.arccos(float(c1))
            beta = np.arccos(float(c2))
            gamma = np.arccos(float(c3))
            # beta = np.arccos(c2)
            # gamma = np.arccos(c3)
            return alpha, beta, gamma

        # Step 0: Extract end-effector position and orientation from request
        px = pose.position.x
        py = pose.position.y
        pz = pose.position.z
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y,pose.orientation.z, pose.orientation.w])

        # ---------------------(1) Base to WC---------------------
        # Step 1: Find the position of wrist center Owc
        pg_0 = Matrix([[px], [py], [pz]])
        p_quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        d_g = s[d7] 
        R0_g = tf.transformations.quaternion_matrix(p_quat)
        R0_g = R0_g[0:3, 0:3]  
        z_g = np.dot(self.R_corr.tolist(), ([[0], [0], [1]]))
        # print("this is zg", z_g)

        # TODO: find wrist center position rwc_0
        #	hint1: we want rwc_0 to be a Matrix type, and we can cast data to matrix type by doing Matrix(data)
        #	hint2: the function np.dot does matrix multiplication for us.
        
        rwc_0 = Matrix(pg_0 - (d_g * (np.dot(R0_g, z_g))))# something
        debug_log("Wrist Center : {}".format(rwc_0.tolist()))
        # Step 2: Calculate theta1 given Owc
        # TODO: find theta1 
        theta1 = atan2(rwc_0[1], rwc_0[0]).evalf()
        # print("theta1:",theta1)

        # Step 3: Define triangle (O2, O3, Owc)
        pO2_0 = Matrix([[s[a1]], [0], [s[d1]]])
        pO2_0 = rot_z(theta1)* pO2_0
        debug_log("        Link 2 position : {}".format(pO2_0.tolist()))

        pO2towc_0 = rwc_0 - pO2_0
        beta_prime = atan2(s[a3], s[d4])

        X2_prime = self.T0_2.subs({th1:theta1, th2:0}).dot([[1], [0], [0], [0]])
        # print("X2_prime:",X2_prime)
        X2_prime = X2_prime[0:3]
        z2_prime = self.T0_2.subs({th1:theta1}).dot([[0], [0], [1], [0]])
        # print("z2_prime:",z2_prime)
        z2_prime = z2_prime[0:3]

        # TODO: find the 3 edge lengths, directions and angles for this triangle
        # 	hint1: all 3 edge lengths can be found with either s[index] or information above
        #	hint2: all 3 edge directions can be interpreted as a vector of unit length
        #   hint3: all 3 angles are derived from equation: A*A+B*B-C*C = 2*A*B*cos(thetaC)
        # 	hint4: functions that might be helpful
        #	       v.normalized(): gives you a unit length vector in v direction
        #	       v.norm(): gives you the length of vector v
        #	       tf.transformations.rotation_matrix(angle,axis): gives you a 4x4 matrix that does angle axis rotation
        #	       np.arccos(float(data)): casting data to "float" ensures precision
        # 	these information will help you solve for theta2 and theta3!
        #	(please add comments to help us understand your code)
        A = s[a2]
        B = pO2towc_0.norm() 
        C = np.sqrt(s[a3]**2+s[d4]**2)
        # print("A type:", type(A))
        # print("B type:", type(B))
        # print("C type:", type(C))
        # print("ABC:",A,B,C)
        alpha, beta, gamma = solveTriangle(A,B,C)
        # print("alpha,beta,gamma:",alpha,beta,gamma)
        b = pO2towc_0.normalized()  
        tmp = copy.deepcopy(pO2towc_0)
        tmp = tmp.row_insert(3,Matrix([[0]]))
        a = (tf.transformations.rotation_matrix(-gamma, z2_prime)*tmp).normalized()[0:3]
        # print("a type:", type(a))
        # TODO: find pO3
        pO3 = pO2_0 + np.array(a)*float(A)# something
        debug_log("        Link 3 position : {}".format(pO3.tolist()))
        c = (rwc_0 - pO3).normalized()

        # Step 4: Find theta2 from triangle
        # TODO: find theta2
        #       hint: the dot product of two vectors has the math form: 
        #	dot(v1,v2) = |v1|*|v2|*cos(theta)
        # 	so the angle theta between the two vectors can be depricted as: 
        #	theta = arccos(dot(v1,v2)/(|v1|*|v2|))

        theta2 = np.arccos(float(np.dot(X2_prime,a)))# something
        
        # Step 5: Find theta3 from triangle
        # TODO: find theta3 
        # 	hint: you can use (something).evalf() to ensure precision
        # what's the difference?
        theta3 = (pi/2-beta-beta_prime).evalf()# (something).evalf()
        # ---------------------(2) WC to end-effector---------------------
        # Step 1: Symbolic form of R3_6 (skip: try offline)
        # You can try doing "(T3_4*T4_5*T5_6)[:3,:3]" and see result.

        # Step 2: Numeric form of R3_6
        R0_3 = self.T0_3[0:3,0:3]
        R3_6 = R0_3.transpose()* Matrix(R0_g)*self.R_corr.transpose()
        R3_6 = R3_6.subs({th1: theta1, th2:theta2, th3: theta3})
        # R3_6 = float(R3_6)

        # Step 3: solve for theta4, theta5 and theta6
        # TODO: in the case of sin(theta5) > 0
        #       hint: make use of "tan2(,).evalf()" function and "R3_6[index]"
        print(R3_6)
        if R3_6[1,2] == 1 :
            debug_log("s5 = 0")
            sum46 = atan2(R3_6[0,1], R3_6[2,1])
            theta4, theta6 = sum46/2, sum46/2
            theta5 = 0
        elif R3_6[1,2] == -1 :
            debug_log("s5 = 0")
            diff46 = atan2(R3_6[0,1], -R3_6[2,1])
            theta4, theta6 = 0, diff46
            theta5 = pi
        else:
            coe = -1
            theta4 = atan2(coe*R3_6[2,2],-coe*R3_6[0,2])# something
            theta6 = atan2(-coe*R3_6[1,1], coe*R3_6[1,0])# something
            theta5 = atan2(coe*sqrt(R3_6[0,2]**2+R3_6[2,2]**2), R3_6[1,2])# something
            print(sin(theta5))
            if sin(theta5)<0:
                # rospy.logerr("LHS:"+str(cos(theta4)**2+(cos(theta4)*the)**2))
                rospy.logerr("s5 < 0")
            elif sin(theta5)<0:
                theta4 = atan2(-coe*R3_6[2,2], coe*R3_6[0,2])# something
                theta6 = atan2(coe*R3_6[1,1], -coe*R3_6[1,0])# something
                theta5 = atan2(-coe*sqrt(R3_6[0,2]**2+R3_6[2,2]**2), R3_6[1,2])# something
                if sin(theta5) > 0:
                    rospy.logerr("s5 > 0")

        # TODO (optional):  in the case of sin(theta5) = 0 ?
        #		    in the case of sin(theta5) < 0 ?
        # (solve for theta4,5,6 in these two situations for extra credit)

        # Step 4: Sanity check
        def reduceEffectiveRange(theta, name):
            if(theta < -pi ):
                debug_log("        {} = {}, being changed to {}".format(name, theta, (theta + 2*pi)))
                return (theta + 2*pi)
            elif(theta > pi):
                debug_log("        {} = {}, being changed to {}".format(name, theta, (theta - 2*pi)))
                return (theta - 2*pi)
            else:
                return theta

        def angleCorrection(theta, old_theta):
            """
                what does this func do?
            """
            d = theta - old_theta
            if d > np.pi:
                rospy.logerr("ANGLE_Correction")
                return theta - 2*np.pi
            elif d < -np.pi:
                rospy.logerr("ANGLE_Correction")
                return theta + 2*np.pi
            else:
                return theta

        theta4 = reduceEffectiveRange(theta4, "theta4")
        theta6 = reduceEffectiveRange(theta6, "theta6")

        theta4 = angleCorrection(theta4, self.old_th4)
        theta6 = angleCorrection(theta6, self.old_th6)

        self.old_theta4 = theta4
        self.old_theta6 = theta6
        
        # Populate response for the IK request
        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = (theta1, theta2, theta3, theta4, theta5, theta6)
        debug_log("    joint angles = {}".format(joint_trajectory_point.positions))
        return joint_trajectory_point

    def verifyIK(self, pose, joint_trajectory_point):

    	debug_log("    verify IK result:")

    	# check result in joint limit range
    	theta = joint_trajectory_point.positions
        checkInRange(rtod(theta[0]), -185, 185, "theta1")
        checkInRange(rtod(theta[1]),  -45,  85, "theta2")
        checkInRange(rtod(theta[2]), -210,  65, "theta3")
        checkInRange(rtod(theta[3]), -350, 350, "theta4")
        checkInRange(rtod(theta[4]), -125, 125, "theta5")
        checkInRange(rtod(theta[5]), -350, 350, "theta6")

    	# extract computed position
        result  = self.performFK([theta[0], theta[1], theta[2], theta[3], theta[4], theta[5]])
        computed_pos = Matrix([[result.position.x], [result.position.y], [result.position.z]])

        # extract requested position
        requested_pos = Matrix([[pose.position.x], [pose.position.y], [pose.position.z]])

    	# error analysis
        error = (requested_pos - computed_pos).norm()

    	# debug log
        debug_log("        EE position received FK: {}".format(requested_pos.tolist()))
        debug_log("        EE position after FK :   {}".format(computed_pos.tolist()))
        debug_log("        EE Error : {}".format(error))

        return error

    def handle_calculate_FK(self, req):
        if len(req.points) < 1: 
            debug_log("\nRECEIVED A CORRUPTED \"my_kuka_FK\" SERVICE REQUEST... EXIT\n")
            return -1
        else:
           debug_log("\nRECEIVED A \"my_kuka_FK\" SERVICE REQUEST!")
        
        # Initialize service response
        ee_pose_list = []

        for x in xrange(0, len(req.points)): 
            debug_log("\n    processing point {}".format(x))
            # extract robot joint angles
            theta_t = req.points[x].positions

            # compute FK
            pose = self.performFK(theta_t)
            ee_pose_list.append(pose)

        debug_log("\nRESPONDED TO \"my_kuka_FK\" SERVICE REQUEST! (length={})".format(len(ee_pose_list)))
        return robot_FKResponse(ee_pose_list)

    def handle_calculate_IK(self, req):
        if len(req.poses) < 1: 
            debug_log("\nRECEIVED A CORRUPTED \"my_kuka_IK\" SERVICE REQUEST... EXIT\n")
            return -1
        else:
            debug_log("\nRECEIVED A \"my_kuka_IK\" SERVICE REQUEST!")
        
        # Initialize service response
        joint_trajectory_list = []
        position_errors = []

        self.old_th4 = 0
        self.old_th6 = 0

        for x in xrange(0, len(req.poses)): 
            debug_log("\n    processing pose {}".format(x))
            # extract robot pose
            pose = req.poses[x]
                    
            # compute IK
            joint_trajectory_point = self.performIK(pose)
            joint_trajectory_list.append(joint_trajectory_point)

            # verify IK accuracy
            error = self.verifyIK(pose, joint_trajectory_point)
            position_errors.append(error)

        debug_log("\nRESPONDED TO \"my_kuka_IK\" SERVICE REQUEST! (length={})".format(len(joint_trajectory_list)))
        return robot_IKResponse(joint_trajectory_list)



if __name__ == "__main__":

    global debug_mode
    debug_mode = True

    debug_log("Step 1: initializing my_kuka solver...")
    rospy.init_node('m_kuka')
    solver = MyKukaSolver()

    debug_log("Step 2: creating a callable proxy to a service")
    # TODO: do this for both FK and IK services
    rospy.Service('FK_service', robot_FK, solver.handle_calculate_FK)
    rospy.Service('IK_service', robot_IK, solver.handle_calculate_IK)

    debug_log("Step 3: ready to receive requests")
    rospy.spin()

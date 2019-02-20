#!/usr/bin/env python

# import modules
import tf
import time
import rospy
import numpy as np
import matplotlib.pyplot as plt

from my_kuka_robot.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sympy import mpmath
from sympy import *

def debug_log(log):
    if (debug_mode):
        print(log)
    pass

def rand_in_range(low, high):
    return low + np.random.rand()*(high - low)

# Radians to Degree
def rtod(q):
    return q*180.0/np.pi

# Degree to Radians
def dtor(q):
    return q*np.pi/180.0

class IKTest(object):

    def __init__(self):
	debug_log("Step 1: initializing test script...")
	# TODO: initialize the ROS node
	#       use the function: rospy.init_node('node_name')
	#       (see the diagram in HW3 2.1.1 to find the argument)

	debug_log("Step 2: waiting for service to be available.")
	# TODO: wait for service to be available
	#       use the function: rospy.wait_for_service('service_name')
	#       (see the diagram in HW3 2.1.1 to find the argument)

	debug_log("Step 3: creating a callable proxy to a service")
	# TODO: declare the service requester role
	#       use the function self.IKSolver = rospy.ServiceProxy('service_name', service_type)
	#       (see the diagram in HW3 2.1.1 to find the argument)

	# the expected response to the predefined test request
	self.IK_RESP = [JointTrajectoryPoint(),
			JointTrajectoryPoint(),
			JointTrajectoryPoint()]
	self.IK_RESP[0].positions = (-3.1233, 0.4022, 0.2960, -0.2039, 2.6062, 1.4159)
	self.IK_RESP[1].positions = (-3.073, 0.9382, -1.4036, -0.9915, 0.9615, -0.4024)
	self.IK_RESP[2].positions = (0.177, 0.5141, 0.5274, -0.3186, 1.4602, 2.1681)

	# predefined test request
	self.IK_REQ = [Pose(),
		       Pose(),
		       Pose()]
	self.IK_REQ[0].position.x = -1.65691213352452
	self.IK_REQ[0].position.y = 0.00101364734482823
	self.IK_REQ[0].position.z = 0.946179234471478
	self.IK_REQ[1].position.x = -2.94615069338
	self.IK_REQ[1].position.y = 0.00657260199077
	self.IK_REQ[1].position.z = 2.07021476465
	self.IK_REQ[2].position.x = 1.43951014220693
	self.IK_REQ[2].position.y = 0.161412812336875
	self.IK_REQ[2].position.z = 0.343112651459669	
	self.IK_REQ[0].orientation.x = -0.69833822    
	self.IK_REQ[0].orientation.y = -0.10371481
	self.IK_REQ[0].orientation.z = -0.01671248
	self.IK_REQ[0].orientation.w = 0.70801671
	self.IK_REQ[1].orientation.x = -0.25579584927
	self.IK_REQ[1].orientation.y = -0.53186509833
	self.IK_REQ[1].orientation.z = 0.772125532224
	self.IK_REQ[1].orientation.w = 0.23560594891
	self.IK_REQ[2].orientation.x = -0.16492813 
	self.IK_REQ[2].orientation.y = -0.42550731  
	self.IK_REQ[2].orientation.z = 0.81557302 
	self.IK_REQ[2].orientation.w = -0.3557849
	pass

    def generate_IK_request(self):
	req = Pose()
	req.position.x = rand_in_range(-2,2)
	req.position.y = rand_in_range(-0.5,0.5)
	req.position.z = rand_in_range(0,2)

	q = tf.transformations.quaternion_from_euler(
		rand_in_range(-3.14,3.14), 
		rand_in_range(-3.14,3.14), 
		rand_in_range(-3.14,3.14), 'ryxz')
	req.orientation.x = q[0]
	req.orientation.y = q[1]
	req.orientation.z = q[2]
	req.orientation.w = q[3]
	
	return req

    def run_tests(self):
	errors = 0
	debug_log("Step 4: ready for testing.")
	
	debug_log("\n    Part 1: predefined tests")
	# TODO: send an IK service request
	#       use the function: response_data = self.IKSolver(request_data)
	#       see "__init__" function above to find the request_data

	errors += self.verify_result(IK_resp)

	debug_log("\n    Part 2: random tests")
	IK_reqs = []
	for i in range(5):
	    pose = self.generate_IK_request()
	    IK_reqs.append(pose)
	
	IK_resp = self.IKSolver(IK_reqs)
	self.show_result(IK_reqs, IK_resp)

	if(errors < 1):
	    debug_log("IK test passed!")
	else:
	    debug_log("IK test failed")

    def show_result(self,IK_reqs, IK_resp):
	for x in xrange(0, len(IK_resp.points)): 
	    pos_r = IK_reqs[x].position
            ori_r = IK_reqs[x].orientation
	    debug_log("        IK_request  {}: position =     {},{},{}".format(x, pos_r.x, pos_r.y, pos_r.z))	
	    debug_log("        IK_request  {}: orientation =  {},{},{},{}".format(x, ori_r.x,ori_r.y,ori_r.z,ori_r.w))
	    debug_log("        IK_response {}: joint angles = {}\n".format(x, IK_resp.points[x].positions))

    def verify_result(self,IK_resp):
	errors = 0
	for x in xrange(0, len(IK_resp.points)): 
	    # TODO: extract the received and expected joint angles response
	    #       theta_r: received joint angle response
	    #       (to access one set of joint angles from the list, you can do: response.points[index].positions)
	    #       theta_e: expected join angle response
	    #       (see "__init__" function to find the expected joint angle response)
	    theta_r = # something
	    theta_e = # something

	    theta_r_mat = Matrix([[theta_r[0]], [theta_r[1]], [theta_r[2]],[theta_r[3]], [theta_r[4]], [theta_r[5]]])
            theta_e_mat = Matrix([[theta_e[0]], [theta_e[1]], [theta_e[2]],[theta_e[3]], [theta_e[4]], [theta_e[5]]])
	    err = (theta_r_mat - theta_e_mat).norm()

	    debug_log("        received IK_response {}: ({})".format(x, theta_r))	
	    debug_log("        expected IK_response {}: ({})".format(x, theta_e))	
            debug_log("        error:   {}\n".format(err))
	    errors += err
  	return errors

if __name__ == "__main__":

    global debug_mode
    debug_mode = True

    my_kuka_IK_test = IKTest()
    my_kuka_IK_test.run_tests()

    rospy.spin()

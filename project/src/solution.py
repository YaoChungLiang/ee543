#!/usr/bin/env python  
import rospy

from std_msgs.msg import Int16
from project1.msg import TwoInts

def callback(data):
	# TODO: compute the sum of data.a and data.b, and save it to "msg"
	msg = Int16(0)
	# print on terminal
	rospy.loginfo(str(data.a) + " + " + str(data.b) + " = " + str(msg))
	
	# TODO: publish "msg" to rostopic publisher "pub"
	

def talker_listener():
	
	rospy.init_node('solution')

	global pub
	# TODO: establish publisher and subscriber relations
	# pub = (publish to a rostopic named "sum", with type Int16, and queue_size=1)
	# (subscribe to a rostopic named "two_ints", with type TwoInts and send to callback function for processing)

	# keep python existing until node is killed
	rospy.spin()


if __name__ == '__main__':
	try:
		talker_listener()
	except rospy.ROSInterruptException:
		raise e

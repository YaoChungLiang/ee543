#!/usr/bin/env python  
import rospy

from std_msgs.msg import Int16
from project1.msg import TwoInts

def callback(data):
	msg = Int16(0)
	msg = data.a + data.b
	rospy.loginfo(str(data.a) + " + " + str(data.b) + " = " + str(msg))
	pub.publish(msg)	

def talker_listener():
	
	rospy.init_node('solution')
	global pub
	pub = rospy.Publisher('sum', Int16, queue_size=1)
	sub = rospy.Subscriber("two_ints", TwoInts, callback)
	rospy.spin()


if __name__ == '__main__':
	try:
		talker_listener()
	except rospy.ROSInterruptException:
		raise e

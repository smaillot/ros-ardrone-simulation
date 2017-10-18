#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from time import time

def takeoff():

	takeoff_publisher = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=10)
	start = time()
	rospy.loginfo('Sending take off command...')
	while not rospy.is_shutdown():
		if time() - start < 3:
			takeoff_publisher.publish(Empty())
		else:
			break

if __name__ == '__main__':
	rospy.init_node('takeoff')
	takeoff()
	rospy.signal_shutdown("Take off succeeded !")
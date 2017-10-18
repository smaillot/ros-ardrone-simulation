#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from time import time

def land():

	land_publisher = rospy.Publisher('/ardrone/land', Empty, queue_size=10)
	start = time()
	rospy.loginfo('Sending land command...')
	while not rospy.is_shutdown():
		if time() - start < 3:
			land_publisher.publish(Empty())
		else:
			break

if __name__ == '__main__':
	rospy.init_node('land')
	land()
	rospy.signal_shutdown("Land succeeded !")
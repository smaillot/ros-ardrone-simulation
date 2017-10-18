#!/usr/bin/env python
import rospy
from time import time
from DroneController import DroneController

def stabilize(controller, timer=-1):

	start = time()
	while not rospy.is_shutdown() and (time == -1 or (time() - start) < timer):
	 
		controller.stop_drone()

if __name__ == '__main__':
	rospy.init_node('stabilize')
	controller = DroneController()
	rospy.loginfo('Stabilizing...')
	stabilize(controller)
	rospy.signal_shutdown("")
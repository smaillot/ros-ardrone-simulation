#!/usr/bin/env python
import rospy
from time import time
from ar_pose.msg import ARMarker
from DroneController import DroneController
from MarkerDetector import MarkerDetector

def align_wall_marker(controller, marker_detector, timer=-1):

	while not rospy.is_shutdown() and controller.get_stab_time() < timer:
	
		controller.stab_time = -1
		while controller.get_stab_time() < 1:
			controller.align_wall_marker(marker_detector, 1, eps=0.3)
		controller.set_speed_rot(controller.stab)
		break

if __name__ == '__main__':
	rospy.init_node('align_wall_marker')
	controller = DroneController()
	marker_detector = MarkerDetector()
	rospy.Subscriber("ar_pose_marker", ARMarker, marker_detector.detect_marker)
	rospy.loginfo('Moving to the wall marker...')
	align_wall_marker(controller, marker_detector)
	rospy.loginfo('moving...')
	rospy.signal_shutdown("")
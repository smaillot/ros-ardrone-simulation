#!/usr/bin/env python
import rospy
from ar_pose.msg import ARMarker
from MarkerDetector import MarkerDetector
from DroneController import DroneController

def align_floor_marker(controller, marker, timer=-1, pose=[0, 0, 1, 0, 0, 0]):

	high = False
	while not rospy.is_shutdown() and controller.get_stab_time() < timer:
	 
		can_see_marker = controller.align_floor_marker(marker, [0, 0, 1, 0], 0.2)
		if not can_see_marker and not high:
			high = True
			controller.go_up(2, 0.2)
	controller.set_speed_rot(controller.stab)

if __name__ == '__main__':
	rospy.init_node('align_floor_marker')
	marker = MarkerDetector()
	controller = DroneController()
	rospy.Subscriber("ar_pose_marker", ARMarker, marker.detect_marker)
	rospy.loginfo('Align with floor marker')
	align_floor_marker(controller, marker)
	rospy.signal_shutdown("")
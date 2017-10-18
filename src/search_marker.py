#!/usr/bin/env python
import rospy
from ar_pose.msg import ARMarker
from DroneController import DroneController
from MarkerDetector import MarkerDetector

def search_marker(controller, marker_detector):

	while not rospy.is_shutdown():
	 
		controller.set_speed_rot(controller.stab_pose([0, 0, 0, 0, 0, 1]))
		controller.publish_command()
		if marker_detector.cam_name == 'front' and marker_detector.detected:
			controller.set_speed_rot(controller.stab)
			controller.publish_command()
			rospy.loginfo('Wall marker detected.')
			break

if __name__ == '__main__':
	rospy.init_node('search_marker')
	controller = DroneController()
	marker_detector = MarkerDetector()
	rospy.Subscriber("ar_pose_marker", ARMarker, marker_detector.detect_marker)
	search_marker(controller, marker_detector)
	rospy.signal_shutdown("")
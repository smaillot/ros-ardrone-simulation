#!/usr/bin/env python
import rospy
from time import time
from ar_pose.msg import ARMarker

from DroneController import DroneController
from MarkerDetector import MarkerDetector
from takeoff import takeoff
from stabilize import stabilize
from toggle_cam import toggle_cam
from align_floor_marker import align_floor_marker
from search_marker import search_marker
from align_wall_marker import align_wall_marker
from land import land

def task(controller, marker):

	# take off
	takeoff()

	# stabilization
	stabilize(controller, 1)

	# align marker 1
	toggle_cam("bottom")
	controller.initialize_stab_time()
	align_floor_marker(controller, marker, 1)

	# search marker 2
	toggle_cam("front")
	search_marker(controller, marker)

	# align marker 2
	controller.initialize_stab_time()
	align_wall_marker(controller, marker, 1)

	# land
	land()

if __name__ == '__main__':
	rospy.init_node('task')
	marker = MarkerDetector()
	controller = DroneController()
	rospy.Subscriber("ar_pose_marker", ARMarker, marker.detect_marker)
	task(controller, marker)
	rospy.signal_shutdown("Task performed !")
#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
import std_srvs.srv
from sensor_msgs.msg import CameraInfo


def toggle_cam(target=""):

	def callback(camera):
		cam_name = ('bottom', 'front')[camera.header.frame_id == 'ardrone_base_frontcam']
		rospy.set_param('/ardrone/cam_name', cam_name)

	def toggle():
		rospy.wait_for_service('/ardrone/togglecam')
		rospy.ServiceProxy('/ardrone/togglecam', std_srvs.srv.Empty)()

	rospy.Subscriber("/ardrone/camera_info", CameraInfo, callback)
	rospy.loginfo(rospy.get_param('/ardrone/cam_name', ''))

	if target == "":
		toggle()
	else:
		if target != rospy.get_param('/ardrone/cam_name', ''):
			toggle()
		

if __name__ == '__main__':
	rospy.init_node('toggle_cam')
	toggle_cam()
	rospy.signal_shutdown("Camera toggled")
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CameraInfo
from time import time

class MarkerDetector(object):
	def __init__(self):
		self.cam_name = ''
		self.detected = False
		self.position = [None, None, None]
		self.orientation = [None, None, None]
		self.last_seq = -1
		self.last_cam = ''
		self.cam_info = rospy.Subscriber("/ardrone/camera_info", CameraInfo, self.detect_camera)
		self.last_toggle = time()

	def detect_marker(self, visualization):
		self.last_cam = self.cam_name
		self.cam_name = ('bottom', 'front')[visualization.header.frame_id == 'ardrone_base_frontcam']
		seq = visualization.header.seq
		if self.last_seq != seq and self.last_seq != -1 and self.last_cam == self.cam_name and time() > 1 + self.last_toggle:
			self.detected = True
		else:
			self.detected = False
		rospy.logdebug('last cam : {}, cam : {}, last seq : {}, seq : {}, detected : {}'.format(self.last_cam, self.cam_name, self.last_seq, seq, self.detected))
		self.last_seq = seq
		self.position = [visualization.pose.pose.position.x, visualization.pose.pose.position.y, visualization.pose.pose.position.z]
		self.orientation = [visualization.pose.pose.orientation.x, visualization.pose.pose.orientation.y, visualization.pose.pose.orientation.z]
		# self.loginfo()

	def detect_camera(self, cam_info):
		rospy.logdebug('Camera_info : ' + str(cam_info.header.frame_id))
		self.last_cam = self.cam_name
		self.cam_name = ('bottom', 'front')[cam_info.header.frame_id == 'ardrone_base_frontcam']
		rospy.logdebug('Camera name saved as ' + str(self.cam_name))

	def togglecam(self, desired_cam):
		rospy.logdebug('Ask for ' + str(desired_cam) + ', current : ' + str(self.cam_name))
		if desired_cam != self.cam_name:
			rospy.logdebug('Switching...')
			self.last_toggle = time()
			togglecam()

	def loginfo(self):
		rospy.loginfo('\nCamera : ' + self.cam_name + '\nPosition\n\tx : ' + str(self.position[0]) + '\n\ty : ' + str(self.position[1]) + '\n\tz : ' + str(self.position[2]) + '\nOrientation\n\tx : ' + str(self.orientation[0]) + '\n\ty : ' + str(self.orientation[1]) + '\n\tz : ' + str(self.orientation[2]))

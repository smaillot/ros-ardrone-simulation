#!/usr/bin/env python
import rospy
from time import time
import math
from geometry_msgs.msg import Twist 


class DroneController(object):

	def __init__(self):
 		self.command = Twist()
 		self.command_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
 		self.stab = [0.05, -0.05, 0, 0, 0, 0] # real [0, 0.1, 0, 0, 0, 0]# sim : [0.05, -0.05, 0, 0, 0, 0]
 		self.stabilized = False
 		self.stab_time = -1

	def stab_pose(self, pose):
		return [pose[i] + self.stab[i] for i in range(6)]

	def stop_drone(self):
		self.set_speed_rot(self.stab)
		self.publish_command()

	def set_speed_rot(self, speed):
		if speed[0] != None:
			self.command.linear.x = speed[0]
		if speed[1] != None:
			self.command.linear.y = speed[1]
		if speed[2] != None:
			self.command.linear.z = speed[2]
		if speed[3] != None:
			self.command.angular.x = speed[3]
		if speed[4] != None:
			self.command.angular.y = speed[4]
		if speed[5] != None:
			self.command.angular.z = speed[5]

	def P_controller(self, measurement, target, gain):
		return - gain * (measurement - target)

	def initialize_stab_time(self):
		self.stab_time = -1

	def align_floor_marker(self, marker, target, eps=0.3):

		if marker.detected:
			K_x = 1
			K_y = 1
			K_z = 2
			K_rz = 0.4
			target_x, target_y, target_z, target_rz = tuple(target)
			y_comm = self.P_controller(marker.position[0], 0, K_x)
			x_comm = self.P_controller(marker.position[1], 0, K_y)
			z_comm = self.P_controller(abs(marker.position[2]), target_z, K_z)
			rz_comm = self.P_controller(marker.orientation[0], 0, K_rz)
			self.set_speed_rot(self.stab_pose([x_comm, y_comm, z_comm, 0, 0, rz_comm]))
			check_dist = math.sqrt((marker.position[0])**2 + (marker.position[1])**2 + (marker.position[2] - target_z)**2)
			rospy.logdebug('Distance from checkpoint : {}\n\tDelta_x = {}\n\tDelta_y = {}\n\tDelta_z = {}\n\tDelta_rz = {}'.format(check_dist, abs(marker.position[0]), abs(marker.position[1]), abs(marker.position[2] - target_z), abs(marker.orientation[0] - target_rz)))
			self.publish_command()

			if check_dist < eps:
				self.stabilized = True
				if self.stab_time == -1:
					self.stab_time = time()
				rospy.logdebug('Stabilized : {}s'.format(self.get_stab_time()))
			else:
				self.stabilized = False
				self.stab_time = -1	
				rospy.logdebug('Stabilizing...')
			return True

		else:
			rospy.logdebug('Marker not detected ! Waiting for it, please move the drone over the marker.\nWill try to go up.')
			self.set_speed_rot(self.stab)
			self.publish_command()
			return False

	def align_wall_marker(self, marker, dist=-1, eps=0.2):

		if marker.detected:
			K_r = 1
			K_z = 1
			K_x = - 1
			r_comm = self.P_controller(marker.position[0], 0, K_r)
			z_comm = self.P_controller(marker.position[1], 0, K_z)
			if dist != -1:
				x_comm = self.P_controller(marker.position[2], dist, K_x)
			else:
				x_comm = 0
			self.set_speed_rot(self.stab_pose([x_comm, 0, z_comm, 0, 0, r_comm]))
			check_dist = math.sqrt((marker.position[0])**2 + (marker.position[1])**2)
			rospy.logdebug('Distance from checkpoint : {}\n\t, x_dist : {}\n\tDelta_x = {}\n\tDelta_y = {}'.format(check_dist, abs(marker.position[2]), abs(marker.position[0]), abs(marker.position[1])))
			self.publish_command()

			if check_dist < eps:
				self.stabilized = True
				if self.stab_time == -1:
					self.stab_time = time()
				rospy.logdebug('Stabilized : {}s'.format(self.get_stab_time()))
			else:
				self.stabilized = False
				self.stab_time = -1	
				rospy.logdebug('Stabilizing...')

		else:
			rospy.logdebug('Marker not detected ! Waiting for it, please move the drone over the marker.')
			self.set_speed_rot(self.stab)
			self.publish_command()
			return False

	def get_stab_time(self):
		if self.stab_time == -1:
			return 0
		else:
			return time() - self.stab_time


	def publish_command(self):

		self.command_publisher.publish(self.command)

	def go_up(self, speed, time):
		self.set_speed_rot([None, None, speed, None, None, None])
		self.publish_command()
		rospy.sleep(time)
		self.stop_drone()
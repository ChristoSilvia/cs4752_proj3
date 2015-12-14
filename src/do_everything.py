#!/usr/bin/env python

# python
import numpy as np
import matplotlib.pyplot as plt
import config
import sys

# ROS
import rospy
from cs4752_proj3.msg import BallPositionVelocity

# Baxter
import baxter_interface

# Pykdl
import baxter_pykdl

# states
BLOCKING = 0
PICKING_UP = 1
THROWING = 2

# calibration parameters
field_center = np.array([0.54, 0.0, -0.08])
throwing_joint_angles = None
blocking_joint_angles = None

# model parameters
no_blocking_cutoff_velocity = 0.01
center_margin = 0.03
blocking_pid_radius = 0.75 * config.goal_width


class Player:
	def __init__(self, limb_name):
		# Initialize Kinematics
		self.limb_name = limb_name
		self.limb = baxter_interface.Limb(limb_name)
		self.limb_kin = baxter_pykdl.baxter_kinematics(limb_name)

		# Subscribe to topics
		rospy.Subscriber('/ball_position_velocity', BallPositionVelocity, self.record_ball_position_velocity)
		self.last_updated = None
		self.ball_position = None
		self.ball_velocity = None
		self.ball_on_our_side = False

		# Iniialize Derived Parameters
		self.goal_center[0] = field_center[0]
		if self.limb_name == "left":
			self.goal_center[1] = field_center[1] + 0.5*config.field_length
		else:
			self.goal_center[1] = field_center[1] - 0.5*config.field_length
		self.goal_center[2] = field_center[2]


		# Initialize State
		self.state = BLOCKING

		while not rospy.is_shutdown():
			if self.state is BLOCKING:
				# check if ball position has not been initialized yet
				if self.ball_position is None:
					continue

				# check if ball velocity is on our side 
				# 	and below the threshold to begin picking up
				if np.linalg.norm(self.ball_velocity) < no_blocking_cutoff_velocity and self.ball_on_our_side:
					self.state = PICKING_UP
					self.exit_control_mode()
					continue

				# check that arm is close to goal radius
				# move there if it is not close enough
				arm_pose = self.limb.endpoint_pose()
				arm_position = config.vector3_to_numpy(arm_pose['position'])
				
				dist_to_goal_center = np.linalg.norm(arm_position - self.goal_center)
				if dist_to_goal_center > blocking_pid_radius:
					self.limb.move_to_joint_positions(blocking_joint_angles)
					initialize_blocker()

				joint_velocities = get_blocking_joint_velocities(self.desired_blocking_position)
				self.limb.set_joint_velocities(
					config.numpy_to_joint_dict(self.limb_name, joint_velocities))
									


				# compute ball position on our wall
					
					
					
				

	def record_ball_position_velocity(self, data):
		self.last_updated = t
		self.ball_position = config.vector3_to_numpy(data.pose.position)
		self.ball_velocity = config.vector3_to_numpy(data.pose.velocity)

		if self.limb_name == "left":
			if self.ball_position[1] > field_center[1] + center_margin:
				self.ball_on_our_side = True
			else:
				self.ball_on_our_side = False
		else:
			if self.ball_position[1] > field_center[1] - center_margin:
				self.ball_on_our_side = False
			else:
				self.ball_on_our_side = True
					
		


if __name__ == '__main__':
	rospy.init_node('game')
	baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()

	try:
		limb_name = rospy.get_param('limb')
	except:
		print("No Limb Parameter")
		if (sys.argv[1] == "left") or (sys.argv[1] == "right"):
			limb_name = sys.argv[1]
	
	if (limb_name == "left" or limb_name == "right"):
		Player(limb_name)

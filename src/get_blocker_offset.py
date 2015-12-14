#!/usr/bin/env python

# python
import numpy as np
import matplotlib.pyplot as plt
import config
import sys

# ROS
import rospy
from cs4752_proj3.msg import BallPositionVelocity, BlockerOffset
from geometry_msgs.msg import Pose

# Baxter
import baxter_interface

# Parameters
no_blocking_cutoff_velocity = 0.02
n_observations_until_considered_stopped = 10

class GetBlockerOffset:
	def __init__(self, limb_name):
		self.limb_name = limb_name
		self.limb = baxter_interface.Limb(limb_name)

		# goal center

		# Iniialize Derived Parameters
		if limb_name == "right":
			self.goal_center = np.array([0.6, -0.5, -0.08])
		else:
			self.goal_center = np.array([0.6, 0.5, -0.08])
		rospy.Subscriber('/goal_center_pose', Pose, self.set_goal_center)
		
		self.gripper_x_offset = 0.0
		self.n_observations_of_stopped_ball = 0
		self.pub = rospy.Publisher('/blocker_offset', BlockerOffset, queue_size=10)
		self.pub.publish(BlockerOffset(0.0, True))

		rospy.Subscriber('/ball_position_velocity', BallPositionVelocity, self.record_ball_position_velocity)

		t_start = rospy.get_time()
		rate = rospy.Rate(60)
		while not rospy.is_shutdown():
			self.pub.publish(BlockerOffset(self.gripper_x_offset, True))
			# if np.sin(rospy.get_time() - t_start) > 0.0:
			# 	self.pub.publish(BlockerOffset(0.1, True))
			# else:
			# 	self.pub.publish(BlockerOffset(-0.1, True))
			rate.sleep()
		# rospy.spin()

	def record_ball_position_velocity(self, data):
		self.last_updated = data.t
		self.ball_position = config.vector3_to_numpy(data.position)
		self.ball_velocity = config.vector3_to_numpy(data.velocity)

		if (self.ball_velocity[1] < no_blocking_cutoff_velocity and self.ball_position[1] > 0.0 and self.limb_name == "left") or (self.ball_velocity[1] > -no_blocking_cutoff_velocity and self.ball_position[1] < 0.0 and self.limb_name == "right"):
			# ball is moving away, so a guard position is the best one
			print("Ball is Stationionary, moving to Guard Position")
			if self.n_observations_of_stopped_ball >= n_observations_until_considered_stopped:
				self.pub.publish(BlockerOffset(self.gripper_x_offset, True))
			else:
				self.pub.publish(BlockerOffset(self.gripper_x_offset, False))
			self.n_observations_of_stopped_ball += 1
			# self.pub.publish(BlockerOffset(0.0))
		else:
			self.n_observations_of_stopped_bal = 0
			print("Ball is moving, intercepting")
			if self.limb_name == "left":
				ball_tan = self.ball_velocity[0]/self.ball_velocity[1]
			else:
				ball_tan = - self.ball_velocity[0]/self.ball_velocity[1]

			y_dist_to_goal = np.abs(self.goal_center[1] - self.ball_position[1]) - config.gripper_depth
	
			no_walls_ball_hit_offset = ball_tan*y_dist_to_goal + (self.ball_position[0] - self.goal_center[0])
	
	
			no_walls_ball_hit_sign = np.sign(no_walls_ball_hit_offset)
			absed_modded_no_walls_ball_hit_offset = np.abs(no_walls_ball_hit_offset) #% 2.0*self.config.field_width
	
			if absed_modded_no_walls_ball_hit_offset > 0.5*config.field_width and absed_modded_no_walls_ball_hit_offset < 1.5*config.field_width:
				ball_hit_offset = no_walls_ball_hit_sign*(config.field_width - absed_modded_no_walls_ball_hit_offset)
			elif absed_modded_no_walls_ball_hit_offset > 1.5*config.field_width:
				ball_hit_offset = no_walls_ball_hit_sign*(2.0*config.field_width - absed_modded_no_walls_ball_hit_offset)
			else:
				ball_hit_offset = no_walls_ball_hit_sign*absed_modded_no_walls_ball_hit_offset
	
	
			# print type(no_walls_ball_hit_location)
			# print no_walls_ball_hit_location
			# self.test_block_pub.publish(Float64(no_walls_ball_hit_location))
	
			self.gripper_x_offset = np.clip(
				ball_hit_offset,
				-0.5*config.goal_width + config.gripper_depth,
				0.5*config.goal_width - config.gripper_depth)

			self.pub.publish(BlockerOffset(self.gripper_x_offset, True))

	def set_goal_center(self, data):
		self.goal_center = config.vector3_to_numpy(data.position)
					
if __name__ == '__main__':
	rospy.init_node('get_blocker_offset')
	baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()

	try:
		limb_name = rospy.get_param('limb')
	except:
		print("No Limb Parameter")
		if (sys.argv[1] == "left") or (sys.argv[1] == "right"):
			limb_name = sys.argv[1]
	
	if (limb_name == "left" or limb_name == "right"):
		GetBlockerOffset(limb_name)

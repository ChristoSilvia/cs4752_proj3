#!/usr/bin/env python


# python
import numpy as np
import matplotlib.pyplot as plt
import config
from copy import deepcopy
import sys

# ROS
import rospy
from geometry_msgs.msg import *
from std_msgs.msg import *

# BAXTER
import baxter_interface

# BAXTER kinematics
from baxter_pykdl import baxter_kinematics

# us
from cs4752_proj3.msg import BlockerOffset
from cs4752_proj3.srv import Reset, ResetResponse



class BlockerJointVelocities():
	def __init__(self, limb_name, center_of_goal):
		self.limb_name = limb_name		
		self.limb = baxter_interface.Limb(limb_name)
		self.limb_kin = baxter_kinematics(limb_name)
		
		self.center_of_goal = center_of_goal 

		endpoint_pose = self.limb.endpoint_pose()
		self.desired_position = config.vector3_to_numpy(endpoint_pose['position'])
		self.desired_orientation = config.quaternion_to_numpy(endpoint_pose['orientation'])


		self.kp = np.array([2.65, 1.3, 0.9, 0.5, 0.5, 0.5])
		self.ki = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
		self.kd = np.array([4.2, 0.0, 0.0, 0.0, 0.0, 0.0])


		self.reset(Reset())

		createService('/reset_blocker', Reset, self.reset, self.limb_name)
		createService('/get_blocker_joint_velocities', BlockingPosition, self.block_ball)
		rospy.Subscriber('/blocker_offset', BlockerOffset, self.handle_offset)
		rospy.spin()
	
	def block_ball(self, args):
		jacobian = np.array(self.limb_kin.jacobian())
		jacobian_pinv = np.linalg.pinv(jacobian)

		endpoint_pose = self.limb.endpoint_pose()
		position = config.vector3_to_numpy(endpoint_pose['position'])
		error[:3] = position - self.desired_position

		orientation = config.quaternion_to_numpy(endpoint_pose['orientation'])
		error[3:] = config.get_angular_error(self.desired_orientation, orientation)

		current_time = rospy.get_time()
		time_interval = current_time - last_time
		integral += error * time_interval
		last_time = current_time
		derivative = (error - last_error)/time_interval
		last_error = error
	
		desired_twist = - self.kp * error - self.ki * integral - self.kd * derivative
	
		joint_velocities = np.dot(jacobian_pinv, desired_twist)

		return BlockingPositionResponse(list(joint_velocities))
	
	def handle_offset(self, data):
		self.desired_position[0] = self.goal_center[0]
		self.desired_position[1] = self.goal_center[1] + data.offset
		self.desired_position[2] = self.goal_center[2]
		  
	def reset(self, args):
		self.integral = np.zeros(6)

		self.start_time = rospy.get_time()
		self.last_time = deepcopy(start_time)
		self.error = np.zeros(6)
		self.last_error = deepcopy(error)
		return ResetResponse()


if __name__ == '__main__':
	rospy.init_node('blocker')
	print("Initialized node 'blocker'")
	baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()

	limb_name = None
	try:
		limb_name = rospy.get_param("limb")
		# pri
	except:
		"no limb param"
	if limb_name is None:
		limb_name = sys.argv[1]
	assert (limb_name == "left" or limb_name == "right")
	print("Initializing Blocker for {0} arm".format(limb_name))
	# POSITION OF GOAL
	left_goal = np.array([0.58,0.64,-0.06])
	right_goal = np.array([0.58, -0.66, -0.06])
	if limb_name == "left":
		Blocker(limb_name, left_goal)
	else:
		Blocker(limb_name, right_goal) 

#!/usr/bin/env python


# python
import numpy as np
import matplotlib.pyplot as plt
import config
import sys
from copy import deepcopy

# ROS
import rospy
from geometry_msgs.msg import *
from std_msgs.msg import *
from cs4752_proj3.srv import MoveRobotResponse, MoveRobot, Action

# BAXTER
import baxter_interface

# BAXTER kinematics
from baxter_pykdl import baxter_kinematics

# us
from cs4752_proj3.msg import *

# goal
goal_position = { 'left_w0': -0.08705,
				  'left_w1':  0.64427,
				  'left_w2': -0.84139,
				  'left_e0':  0.07248,
				  'left_e1':  1.33341,
				  'left_s0': -0.12502,
				  'left_s1': -0.49011 }


drastic_movement_dist = 1.0

class Blocker():
	def __init__(self, limb_name, center_of_goal):
		self.limb_name = limb_name		
		self.limb = baxter_interface.Limb(limb_name)
		self.joint_names = self.limb.joint_names()
		self.limb_kin = baxter_kinematics(limb_name)

		self.field_width = 0.6858
		self.field_length = 1.3843
		self.goal_width = 0.29845
		self.gripper_width = 0.05
		self.gripper_depth = 0.03

		self.y_velocity_cutoff = 5e-3
		self.joint_position_tolerance = 0.02 
		
		self.center_of_goal = center_of_goal

		# BEGINNING OF IK

		if self.limb_name == "left":
		 	theta = -0.25*np.pi
		else:
		 	theta = 0.25*np.pi

		self.blocking_orientation = np.array([np.sin(theta), np.cos(theta), 0.0, 0.0])

		self.goal_joint_values = None
		while self.goal_joint_values == None and not rospy.is_shutdown():
			print("Trying IK")
			self.goal_joint_values = np.array(self.limb_kin.inverse_kinematics(
				list(self.center_of_goal),
				orientation=list(self.blocking_orientation),
				seed=list(config.joint_dict_to_numpy(self.limb_name, self.limb.joint_angles()))))
		print("Found IK Solution")
		# print(self.goal_joint_values)

		# # eps = 1e-3
		# # joint_angle_error_index = np.argmin([np.abs(goal_joint_values[6] - 0.5*np.pi), 
		# # 	np.abs(goal_joint_values[6] + 0.5*np.pi)])
		# # joint_angle_error = [goal_joint_values[6] - 0.5*np.pi, goal_joint_values[6] + 0.5*np.pi][joint_angle_error_index]
		# # print(goal_joint_values)
		# # print("Joint Angle Error is: {0}".format(joint_angle_error))
		# # while not rospy.is_shutdown() and np.abs(joint_angle_error) > self.joint_position_tolerance:
		# # 	print("Joint Angle Error is: {0}".format(joint_angle_error))
		# # 	jacobian = np.array(self.limb_kin.jacobian(joint_values=config.numpy_to_joint_dict(self.limb_name, goal_joint_values)))
		# # 	null_joint_movement = config.null(jacobian)[1][:,0]

		# # 	minimizing_joint_velocity = - null_joint_movement * (joint_angle_error/null_joint_movement[6])
		# # 	goal_joint_values += minimizing_joint_velocity
		# # 	joint_angle_error_index = np.argmin([np.abs(goal_joint_values[6] - 0.5*np.pi), 
		# # 		np.abs(goal_joint_values[6] + 0.5*np.pi)])
		# # 	joint_angle_error = [goal_joint_values[6] - 0.5*np.pi, goal_joint_values[6] + 0.5*np.pi][joint_angle_error_index]
		# # print("Finished Minimizing Joint Angle Error")

		# #self.limb.move_to_joint_positions(goal_position)

				# END OF IK

		endpoint_pose = self.limb.endpoint_pose()
		self.desired_position = config.vector3_to_numpy(endpoint_pose['position'])
		self.desired_orientation = config.quaternion_to_numpy(endpoint_pose['orientation'])

		rospy.Subscriber(
			'/blocker_offset', 
			BlockerOffset, 
			self.handle_offset_update)

		config.createService('block', Action, self.block, "")
		
		self.cancel_sub = rospy.Subscriber("/action_cancel", Int32, self.cancel, queue_size=1)

		self.kp_drastic = np.array([2.65, 1.3, 0.9])
		self.ki_drastic = np.array([0.0, 0.0, 0.0])
		self.kd_drastic = np.array([4.2, 0.0, 0.0])

		# self.kp = np.array([2.65, 1.3, 0.9, 5.0, 5.0, 5.0])
		# self.ki = np.array([0.0, 1.5, 0.4, 0.0, 0.0, 0.0])
		# self.kd = np.array([4.2, 0.0, 0.0, 0.0, 0.0, 0.0])
		self.kp = np.array([2.65, 1.3, 0.9, 5.0, 5.0, 5.0])
		self.ki = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
		self.kd = np.array([4.2, 0.0, 0.0, 0.0, 0.0, 0.0])

		self.null_norm_squared = 0.1**2

		self.block(Action())

		rospy.spin()

	def cancel(self, msg):
		if msg.data == BLOCK:
			self.canceled = True

	def block(self, req):
		self.moving_away = False
		self.canceled = False


		# COMMENTED OUT FOR TESTING
		# self.limb.move_to_joint_positions(
		# 	config.numpy_to_joint_dict(self.limb_name, self.goal_joint_values),
		# 	threshold=self.joint_position_tolerance)



		integral = np.zeros(6)
		start_time = rospy.get_time()
		self.start_time = rospy.get_time()
		last_time = start_time
		error = np.zeros(6)
		last_error = error
		while not rospy.is_shutdown() and not self.moving_away:
			
			if self.canceled:
				return ActionResponse(False)


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
	

			if np.linalg.norm(error[:3]) > drastic_movement_dist:
				print("Drastic")
				velocity_jacobian = np.array(self.limb_kin.jacobian())[:3,:]
				velocity_jacobian_pinv = np.linalg.pinv(velocity_jacobian)

				desired_twist = - self.kp_drastic * error[:3] - self.ki_drastic * error[:3] - self.kd_drastic * error[:3]
				joint_velocities = np.dot(velocity_jacobian_pinv, desired_twist)
			else:
				print("Not Drastic")
				desired_twist = - self.kp * error - self.ki * integral - self.kd * derivative
	
				jacobian = np.array(self.limb_kin.jacobian())
				jacobian_null_vector = config.null(jacobian)[1][:,0]
				jacobian_pinv = np.linalg.pinv(jacobian)

				eps = 1e-5
				joint_dict = self.limb.joint_angles()
				joint_dicts = []
				for i in xrange(7):
					joint_dicts.append(deepcopy(joint_dict))
					joint_dicts[i][self.limb_name+"_"+config.joint_names[i]] += eps

				direction_of_manipulability = config.direction_of_manipulability(jacobian, [self.limb_kin.jacobian(joint_values=joint_dicts[i]) for i in xrange(7)], eps)
				
				bare_joint_velocities = np.dot(jacobian_pinv, desired_twist)

				t_null = config.maximize_cosine_constrained(jacobian_null_vector, bare_joint_velocities, direction_of_manipulability, self.null_norm_squared)

				joint_velocities = bare_joint_velocities + t_null * jacobian_null_vector
				# joint_velocities = bare_joint_velocities
				
			print(joint_velocities)
			# self.limb.set_joint_velocities(
			# 	config.numpy_to_joint_dict(self.limb_name, joint_velocities))

		return ActionResponse(True)

	def handle_offset_update(self, data):
		self.desired_position[0] = self.center_of_goal[0] + data.offset
		self.desired_position[1] = self.center_of_goal[1] 
		self.desired_position[2] = self.center_of_goal[2]


if __name__ == '__main__':
	rospy.init_node('blocker')
	print("Initialized node 'blocker'")
	baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()

	limb_name = None
	try:
		limb_name = rospy.get_param("limb")
	except:
		"no limb param"
	if limb_name is None:
		limb_name = sys.argv[1]
	assert (limb_name == "left" or limb_name == "right")
	print("Initializing Blocker for {0} arm".format(limb_name))
	# POSITION OF GOAL
	left_goal = np.array([0.58,0.64,-0.06])
	right_goal = np.array([0.58, -0.65, -0.06])
	if limb_name == "left":
		Blocker(limb_name, left_goal)
	else:
		Blocker(limb_name, right_goal) 

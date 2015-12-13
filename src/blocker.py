#!/usr/bin/env python


# python
import numpy as np
import matplotlib.pyplot as plt
import config
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
from cs4752_proj3.msg import *

# goal
goal_position = { 'left_w0': -0.08705,
				  'left_w1':  0.64427,
				  'left_w2': -0.84139,
				  'left_e0':  0.07248,
				  'left_e1':  1.33341,
				  'left_s0': -0.12502,
				  'left_s1': -0.49011 }


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
			theta = 0.75*np.pi

		self.blocking_orientation = np.array([np.sin(theta), np.cos(theta), 0.0, 0.0])

		goal_joint_values = None
		while goal_joint_values == None and not rospy.is_shutdown():
			print("Trying IK")
			goal_joint_values = np.array(self.limb_kin.inverse_kinematics(
				list(self.center_of_goal),
				orientation=list(self.blocking_orientation),
				seed=list(config.joint_dict_to_numpy(self.limb_name, self.limb.joint_angles()))))
		print("Found IK Solution")
		print(goal_joint_values)

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
		print("Beginning to Move to Goal Pose")
		self.limb.move_to_joint_positions(
			config.numpy_to_joint_dict(self.limb_name, goal_joint_values),
			threshold=self.joint_position_tolerance)
		print("Completed Moving to Goal Pose")

		# END OF IK

		endpoint_pose = self.limb.endpoint_pose()
		self.desired_position = config.vector3_to_numpy(endpoint_pose['position'])
		self.desired_orientation = config.quaternion_to_numpy(endpoint_pose['orientation'])

		rospy.Subscriber(
			'/ball_position_velocity', 
			BallPositionVelocity, 
			self.handle_position_velocity)


		self.kp = np.array([2.65, 1.3, 0.9, 0.5, 0.5, 0.5])
		self.ki = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
		self.kd = np.array([4.2, 0.0, 0.0, 0.0, 0.0, 0.0])

		ts = []
		self.ts = []
		xs = []
		ys = []
		zs = []
		bs = []
		self.unnormalized_ball_poses = []
		self.reflected_ball_poses = []

		integral = np.zeros(6)

		start_time = rospy.get_time()
		self.start_time = rospy.get_time()
		last_time = start_time
		error = np.zeros(6)
		last_error = error
		while not rospy.is_shutdown()  :
			jacobian = np.array(self.limb_kin.jacobian())
			jacobian_pinv = np.linalg.pinv(jacobian)

			position = config.vector3_to_numpy(self.limb.endpoint_pose()['position'])
			error[:3] = position - self.desired_position

			orientation = config.quaternion_to_numpy(endpoint_pose['orientation'])
			relative_orientation = config.multiply_quaternion(orientation, 
				config.invert_unit_quaternion(self.desired_orientation))
			relative_orientation_angle = 2.0*np.arccos(relative_orientation[3])
			if relative_orientation_angle < 1e-6:
				error[3:] = np.zeros(3)
			else:
				relative_orientation_axis = relative_orientation[:3]/np.sin(0.5*relative_orientation_angle)
				orientation_error = relative_orientation_angle*relative_orientation_axis
				error[3:] = orientation_error
			
			bs.append(self.desired_position[0] - self.center_of_goal[0])

			# print "bs"
			# print type(bs[0])
			# print len(bs)

			current_time = rospy.get_time()
			time_interval = current_time - last_time
			integral += error * time_interval
			last_time = current_time
			derivative = (error - last_error)/time_interval
			last_error = error
	
			ts.append(current_time - start_time)
			xs.append(error[0])
			ys.append(error[1])
			zs.append(error[2])
	
			desired_twist = np.empty(6)
			desired_twist = - self.kp * error - self.ki * integral - self.kd * derivative
	
			joint_velocities = np.dot(jacobian_pinv, desired_twist)
	
			self.limb.set_joint_velocities(
				config.numpy_to_joint_dict(self.limb_name, joint_velocities))


		plt.figure()
		plt.plot(ts, xs, color="red")
		plt.plot(ts, ys, color="green")
		plt.plot(ts, zs, color="blue")
		plt.grid(True)
		plt.figure()
		plt.plot(ts, bs, color="brown")
		plt.plot(self.ts, self.unnormalized_ball_poses, color="orange")
		plt.plot(self.ts, self.reflected_ball_poses, color="black")
		plt.axis([np.min(self.ts), np.max(self.ts), np.min(self.reflected_ball_poses)-0.05, np.max(self.reflected_ball_poses)+0.05])
		plt.grid(True)
		plt.show()
		rospy.spin()

	def handle_position_velocity(self, data):
		# print("Recieved Data")
		# print("=============")
		ball_position = config.vector3_to_numpy(data.position)
		ball_velocity = config.vector3_to_numpy(data.velocity)
		# print("Ball Position: {0}".format(ball_position))
		# print("Ball Velocity: {0}".format(ball_velocity))
		
		self.ts.append(rospy.get_time() - self.start_time)

		if (ball_velocity[1] < self.y_velocity_cutoff and self.limb_name == "left") or (ball_velocity[1] > -self.y_velocity_cutoff and self.limb_name == "right"):
		# if False:
			# print("Ball is Moving Away, going to Guard Position")
			self.unnormalized_ball_poses.append(self.center_of_goal[0])
			self.reflected_ball_poses.append(self.center_of_goal[0])
			self.desired_position[0] = self.center_of_goal[0]
			self.desired_position[1] = self.center_of_goal[1]
			self.desired_position[2] = self.center_of_goal[2]
		else:
			if self.limb_name == "left":
				ball_tan = ball_velocity[0]/ball_velocity[1]

			else:
				ball_tan = ball_velocity[0]/(-ball_velocity[1])

			# print ball_tan

			dist_to_goal = np.abs(self.center_of_goal[1] - ball_position[1]) - self.gripper_depth
			# print("Ball Y Distance to Goal: {0}".format(dist_to_goal))

			no_walls_ball_hit_offset = ball_tan*dist_to_goal + (ball_position[0] - self.center_of_goal[0])


			self.unnormalized_ball_poses.append(no_walls_ball_hit_offset + self.center_of_goal[0])

			no_walls_ball_hit_sign = np.sign(no_walls_ball_hit_offset)
			absed_modded_no_walls_ball_hit_offset = np.abs(no_walls_ball_hit_offset) #% 2.0*self.field_width

			if absed_modded_no_walls_ball_hit_offset > 0.5*self.field_width and absed_modded_no_walls_ball_hit_offset < 1.5*self.field_width:
				ball_hit_offset = no_walls_ball_hit_sign*(self.field_width - absed_modded_no_walls_ball_hit_offset)
			elif absed_modded_no_walls_ball_hit_offset > 1.5*self.field_width:
				ball_hit_offset = no_walls_ball_hit_sign*(2.0*self.field_width - absed_modded_no_walls_ball_hit_offset)
			else:
				ball_hit_offset = no_walls_ball_hit_sign*absed_modded_no_walls_ball_hit_offset


			# print type(no_walls_ball_hit_location)
			# print no_walls_ball_hit_location
			# self.test_block_pub.publish(Float64(no_walls_ball_hit_location))

			gripper_x_offset = np.clip(
				ball_hit_offset,
				-0.5*self.goal_width + self.gripper_depth,
				0.5*self.goal_width - self.gripper_depth)

			self.reflected_ball_poses.append(gripper_x_offset + self.center_of_goal[0])

			# self.desired_position = self.center_of_goal
			self.desired_position[0] = self.center_of_goal[0] + gripper_x_offset
			self.desired_position[1] = self.center_of_goal[1]
			self.desired_position[2] = self.center_of_goal[2]
		# print("Desired Position: {0}".format(self.desired_position))
			

		  

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

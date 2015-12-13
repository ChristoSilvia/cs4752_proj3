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
		self.gripper_width = 0.04
		self.gripper_depth = 0.03

		self.y_velocity_cutoff = 5e-3
		self.joint_position_tolerance = 0.02
		
		self.center_of_goal = center_of_goal

		# BEGINNING OF IK

		# if self.limb_name == "left":
		# 	wrist_angle = -0.25*np.pi
		# else:
		# 	wrist_angle = 0.75*np.pi
		# self.blocking_orientation = np.array([np.sin(wrist_angle), np.cos(wrist_angle), 0.0, 0.0])

		# goal_joint_values = None
		# while goal_joint_values is None:
		# 	print("Trying IK")
		# 	goal_joint_values = self.limb_kin.inverse_kinematics(
		# 		list(self.center_of_goal),
		# 		orientation=list(self.blocking_orientation),
		# 		seed=list(config.joint_dict_to_numpy(self.limb_name, self.limb.joint_angles())))
			
		# self.limb.move_to_joint_positions(goal_position)
		# self.limb.move_to_joint_positions(
		# 	goal_joint_values,
		# 	threshold=self.joint_position_tolerance)

		# END OF IK

		self.desired_position = config.vector3_to_numpy(self.limb.endpoint_pose()['position'])

		rospy.Subscriber(
			'/ball_position_velocity', 
			BallPositionVelocity, 
			self.handle_position_velocity)

		self.test_block_pub = rospy.Publisher('/test_block_pos', Float64,queue_size=10) 

		# x oscillates at 5.3
		self.kp = np.array([2.65, 1.3, 0.9])
		self.ki = np.array([0.0, 1.5, 0.4])
		self.kd = np.array([4.2, 0.0, 0.0])

		ts = []
		xs = []
		ys = []
		zs = []
		bs = []

		integral = np.zeros(3)

		start_time = rospy.get_time()
		last_time = start_time
		last_error = config.vector3_to_numpy(self.limb.endpoint_pose()['position']) - self.desired_position
		while not rospy.is_shutdown()  :
			jacobian = np.array(self.limb_kin.jacobian())
			jacobian_pinv = np.linalg.pinv(jacobian)
	
			position = config.vector3_to_numpy(self.limb.endpoint_pose()['position'])
			error = position - self.desired_position
			
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
			desired_twist[0:3] = - self.kp * error - self.ki * integral - self.kd * derivative
			desired_twist[3:6] = np.zeros(3)
	
			joint_velocities = np.dot(jacobian_pinv, desired_twist)
	
			# self.limb.set_joint_velocities(
			# 	config.numpy_to_joint_dict(self.limb_name, joint_velocities))


		plt.figure()
		plt.plot(ts, xs, color="red")
		plt.plot(ts, ys, color="green")
		plt.plot(ts, zs, color="blue")
		plt.grid(True)
		plt.figure()
		plt.plot(ts, bs, color="brown")
		plt.grid(True)
		plt.show()

	def handle_position_velocity(self, data):
		print("Recieved Data")
		print("=============")
		ball_position = config.vector3_to_numpy(data.position)
		ball_velocity = config.vector3_to_numpy(data.velocity)
		print("Ball Position: {0}".format(ball_position))
		print("Ball Velocity: {0}".format(ball_velocity))
		
		if (ball_velocity[1] < self.y_velocity_cutoff and self.limb_name == "left") or (ball_velocity[1] > -self.y_velocity_cutoff and self.limb_name == "right"):
		# if False:
			print("Ball is Moving Away, going to Guard Position")
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

			no_walls_ball_hit_location = ball_tan*dist_to_goal + ball_position[0]

			# print type(no_walls_ball_hit_location)
			# print no_walls_ball_hit_location
			# self.test_block_pub.publish(Float64(no_walls_ball_hit_location))

			gripper_x_target = np.clip(
				no_walls_ball_hit_location,
				self.center_of_goal[0] - 0.5*self.goal_width - self.gripper_depth,
				self.center_of_goal[0] + 0.5*self.goal_width + self.gripper_depth)

			# print gripper_x_target

			# self.desired_position = self.center_of_goal
			self.desired_position[0] = gripper_x_target
			self.desired_position[1] = self.center_of_goal[1]
			self.desired_position[2] = self.center_of_goal[2]
		print("Desired Position: {0}".format(self.desired_position))
			

		  

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
	left_goal = np.array([0.592,0.641,-0.0687])
	Blocker(limb_name, left_goal) 

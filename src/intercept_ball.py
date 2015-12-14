#!/usr/bin/env python


import rospy

from copy import deepcopy
import sys
import numpy as np
import config

import baxter_interface

class Interceptor:
	def __init__(self, limb_name, center_of_field):

		# positions are in global frame
		# angles are about the z axis, right-handed, and zero at the x axis.
		# GUESSES
		self.field_width = 0.68
		self.field_length = 1.38
		self.ball_radius = 0.015

		target_position = np.array([0.5, -0.2])
		target_angle = np.pi/6.0
		target_velocity = np.array([0.1, 0.2])

		arm_speed = 0.6
		arm_startup_time = 0.1
		arm_distance_margin = 0.06

		self.center_of_field = center_of_field
		self.min_x = self.center_of_field - 0.5*self.field_width + arm_distance_margin
		self.max_x = self.center_of_field + 0.5*self.field_width - arm_distance_margin
		self.min_y = self.center_of_field - 0.5*self.field_length + arm_distance_margin
		self.max_y = self.center_of_field + 0.5*self.field_length - arm_distance_margin

		self.limb = baxter_interface.Limb(limb_name)
		limb_location = config.vector3_to_numpy(self.limb.endpoint_pose()['position'])
		t_intercept = min_intercept_time(arm_startup_time,
			arm_speed,
			limb_location[:2],
			target_velocity,
			target_position,
			0.7,
			0.01)

		if target_velocity[0] > 0:
			t_collide_with_flat_side = (self.max_x - target_position[0])/target_velocity[0]
		elif target_velocity[0] < 0:
			t_collide_with_flat_side = (target_position[0] - self.min_x)/target_velocity[0]
		else:
			t_collide_with_flat_side = float('inf')

		if target_velocity[1] > 0:
			t_collide_with_goal_side = (self.max_y - target_position[1])/target_velocity[1]
		elif target_velocity[1] < 0:
			t_collide_with_goal_side = (target_position[1] - self.min_y)/target_velocity[1]
		else:
			t_collide_with_goal_side = float('inf')

		if t_intercept < min(t_collide_with_flat_side, t_collide_with_goal_side):
			print("Found a Move")
			x_after_bounce =		
			

		print(t_intercept)
		x_intercept = target_position + t_intercept*target_velocity
		print(x_intercept)





def min_intercept_time(arm_startup_time, arm_speed, arm_position, ball_velocity, ball_position, t_guess, tol):
	t = deepcopy(t_guess)
	x_dist_term = (ball_position[0] - arm_position[0]) + ball_velocity[0]*t
	y_dist_term = (ball_position[1] - arm_position[1]) + ball_velocity[1]*t
	d_of_t = np.sqrt(x_dist_term**2 +y_dist_term**2)
	normalized_time = t - arm_startup_time
	arm_movement_term = arm_speed/normalized_time
	g_of_t = d_of_t - arm_movement_term 
	while np.abs(g_of_t) > tol:
		t = t - g_of_t / ( (x_dist_term*ball_velocity[0] + y_dist_term*ball_velocity[1])/d_of_t + arm_movement_term/normalized_time )
		x_dist_term = (ball_position[0] - arm_position[0]) + ball_velocity[0]*t
		y_dist_term = (ball_position[1] - arm_position[1]) + ball_velocity[1]*t
		d_of_t = np.sqrt(x_dist_term**2 +y_dist_term**2)
		normalized_time = t - arm_startup_time
		arm_movement_term = arm_speed/normalized_time
		g_of_t = d_of_t - arm_movement_term 
	return t

	

if __name__ == '__main__':
	rospy.init_node('interceptor')
	baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()

	try:
		limb_name = rospy.get_param('limb_name')
	except KeyError:
		if sys.argv[1] == "left" or sys.argv[1] == "right":
			limb_name = sys.argv[1]
		else:
			assert False
	print("Initializing Ball Interception on limb {0}".format(limb_name))
	Interceptor(limb_name)

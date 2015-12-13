#!/usr/bin/env python


import rospy

from copy import deepcopy

import baxter_interface

class Interceptor:
	def __init__(self, limb_name):

		# positions are in global frame
		# angles are about the z axis, right-handed, and zero at the x axis.
		target_position = np.array([0.5, 0.6, -0.8])
		target_angle = np.pi/6.0


def min_intercept_time(arm_startup_time, arm_speed, arm_position, ball_velocity, ball_position, t_guess, tol):
	t = deepcopy(t_guess)
	x_dist_term = (ball_position[0] - arm_position[0]) + ball_velocity[0]*t
	y_dist_term = (ball_position[1] - arm_position[1]) + ball_velocity[1]*t
	d_of_t = np.sqrt(x_dist_term**2 +y_dist_term**2)
	normalized_time = t - arm_start_time
	arm_movement_term = arm_speed/normalized_time
	g_of_t = d_of_t - arm_movement_term 
	while g_of_t > tol:
		t = t - g_of_t / ( (x_dist_term*ball_velocity[0] + y_dist_term*ball_velocity[1])/d_of_t + arm_movement_term/normalized_time )
		x_dist_term = (ball_position[0] - arm_position[0]) + ball_velocity[0]*t
		y_dist_term = (ball_position[1] - arm_position[1]) + ball_velocity[1]*t
		d_of_t = np.sqrt(x_dist_term**2 +y_dist_term**2)
		normalized_time = t - arm_start_time
		arm_movement_term = arm_speed/normalized_time
		g_of_t = d_of_t - arm_movement_term 
	return t

	

if __name__ == '__main__':
	rospy.init_node('interceptor')
	baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()

	limb_name = rospy.get_param('limb_name')
    if limb_name is None:
        if sys.argv[1] == "left" or sys.argv[1] == "right":
            limb_name = sys.argv[1]
        else:
            assert False "No limb"

	print("Initializing Ball Interception on limb {0}".format(limb_name))
    Interception(limb_name)

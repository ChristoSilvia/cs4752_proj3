#!/usr/bin/env python

# python
import numpy as np
import config

# ROS
import rospy
from geometry_msgs.msg import Point, PoseStamped, Pose
from cs4752_proj3.msg import BallPositionVelocity

virtual_ball_position_saturation = np.array([0.7, 1.2])
virtual_ball_velocity_saturation = np.array([0.3, 1.0])
virtual_ball_speed = 0.005
virtual_ball_acceleration = 0.001

center_of_field = np.array([0.6, 0.0, -0.08])
virtual_ball_speed = 0.1 
virtual_ball_acceleration = 0.1
ball_bounce_zone = 0.1

class DetermineVelocities:
	def __init__(self):
		rospy.init_node('ball_position_velocity_kalman_filter')
		print("Initialized node DetermineVelocities")

		self.max_past_data_entries = 10
		self.state = None
		self.previous_time = None

		self.center_of_field = np.array([0.56, 0.0, -0.08])
		rospy.Subscriber('/field_center_pose', Pose, self.set_field_center)

		self.pub = rospy.Publisher('/ball_position_velocity', BallPositionVelocity, 
			queue_size=10) 
		rospy.Subscriber('/ball_pose_kinect', PoseStamped, self.handle_data)

		rospy.spin()

	def handle_data(self, data):
		# print("Recieved Data")
		t = data.header.stamp.secs + 1e-9*data.header.stamp.nsecs
		position = np.array([data.pose.position.x, data.pose.position.y])
		# print("Ball Position: {0}".format(position))
		state = np.array([data.pose.position.x, data.pose.position.y, 0.0, 0.0])

		if self.state is None:
			self.state = np.zeros(4)
			self.state[:2] = position
			self.covariance = self.state_no_info_covariance
			self.previous_position = position
			self.previous_time = t
		else:
			delta_t = t - self.previous_time
			self.previous_time = t

			update_matrix = np.array([
				[1.0, 0.0, delta_t, 0.0],
				[0.0, 1.0, 0.0, delta_t],
				[0.0, 0.0, 1.0, 0.0],
				[0.0, 0.0, 0.0, 1.0]])

			model_noise_matrix = np.array([
			[((2.0/np.pi)*virtual_ball_position_saturation[0]*np.arctan((2.0/np.pi)*(virtual_ball_speed/virtual_ball_position_saturation[0])*delta_t))**2, 0.0, 0.0, 0.0],
				[0.0, ((2.0/np.pi)*virtual_ball_position_saturation[1]*np.arctan((2.0/np.pi)*(virtual_ball_speed/virtual_ball_position_saturation[1])*delta_t))**2, 0.0, 0.0],
				[0.0, 0.0, ((2.0/np.pi)*virtual_ball_velocity_saturation[0]*np.arctan((2.0/np.pi)*(virtual_ball_acceleration/virtual_ball_velocity_saturation[0])*delta_t))**2, 0.0],
				[0.0, 0.0, 0.0, ((2.0/np.pi)*virtual_ball_velocity_saturation[1]*np.arctan((2.0/np.pi)*(virtual_ball_acceleration/virtual_ball_velocity_saturation[1])*delta_t))**2]])
			# model_noise_matrix = np.array([
			# 	[0.05**2, 0.0, 0.0, 0.0],
				# [0.0, 0.05**2, 0.0, 0.0],
				# [0.0, 0.0, 0.01**2, 0.0],
				# [0.0, 0.0, 0.0, 0.01**2]])

			predicted_position = self.state[:2] + delta_t * self.state[2:]
			# check for collision with a top wall
			if (predicted_position[0] > center_of_field[0] + 0.5*config.field_width - config.ball_radius - ball_bounce_zone) or (predicted_position[0] < center_of_field[0] - 0.5*config.field_width + config.ball_radius + ball_bounce_zone):
				update_matrix[2,2] = 0.0
				model_noise_matrix[2,2] += self.state[2]**2
				print("Bounce off Top/Bottom")
		
			# check for collision with a bottom wall
			if (predicted_position[1] > center_of_field[1] + 0.5*config.field_length - config.ball_radius - ball_bounce_zone) or (predicted_position[1] < center_of_field[1] - 0.5*config.field_length + config.ball_radius + ball_bounce_zone):
				update_matrix[3,3] = 0.0
				model_noise_matrix[3,3] += self.state[3]**2
				print("Bounce off Left/Right")
			print("x: {0} < {1} < {2}".format(center_of_field[0] - 0.5*config.field_width + config.ball_radius + ball_bounce_zone, predicted_position[0], center_of_field[0] + 0.5*config.field_width - config.ball_radius - ball_bounce_zone))
			print("y: {0} < {1} < {2}".format(center_of_field[1] - 0.5*config.field_length + config.ball_radius + ball_bounce_zone, predicted_position[1],center_of_field[1] + 0.5*config.field_length - config.ball_radius - ball_bounce_zone ))

			# print("Update Matrix: {0}".format(update_matrix))

			offset = np.array([0.0, 0.0, 0.0, 0.0])

			state_to_measurement_matrix = np.array([
				[1.0, 0.0, 0.0, 0.0],
				[0.0, 1.0, 0.0, 0.0],
				[1.0, 0.0, -delta_t, 0.0],
				[0.0, 1.0, 0.0, -delta_t]])
			
			# predict
			predicted_state = np.dot(update_matrix, self.state) + offset
			predicted_covariance = np.dot(update_matrix,np.dot(self.covariance, update_matrix.T)) + model_noise_matrix
			# print("Predicted State: {0}".format(predicted_state))
			# print("Predicted Covariance: {0}".format(predicted_covariance))
			
			# compute error
			measurement = np.empty(4)
			measurement[:2] = position
			measurement[2:] = self.previous_position
			# print("Measured State: {0}".format(measurement))
			error_in_mean = measurement - np.dot(state_to_measurement_matrix, predicted_state)
			error_in_covariance = np.dot(state_to_measurement_matrix, 
				np.dot(predicted_covariance, state_to_measurement_matrix.T)) + self.measurement_noise

			# compute kalman gain
			kalman_gain = np.dot(predicted_covariance, 
				np.dot(state_to_measurement_matrix.T, np.linalg.inv(error_in_covariance)))

			self.state = predicted_state + np.dot(kalman_gain, error_in_mean)
			self.covariance = np.dot(np.eye(4) - np.dot(kalman_gain, state_to_measurement_matrix), predicted_covariance)
			
			self.pub.publish(BallPositionVelocity(t, Point(self.state[0], self.state[1], 0), Point(self.state[2], self.state[3], 0)))
			self.previous_position = position
			# print("Resultant State: {0}".format(self.state))

	state_no_info_covariance = np.array(
		[[0.03 ** 2, 0.0, 0.0, 0.0],
		[0.0, 0.03 ** 2, 0.0, 0.0],
		[0.0, 0.0, 0.5 ** 2, 0.0],
		[0.0, 0.0, 0.0, 0.5 ** 2]])

	measurement_noise = np.array(
		[[0.005**2, 0.0, 0.0, 0.0],
		[0.0, 0.005**2, 0.0, 0.0],
		[0.0, 0.0, 0.005**2, 0.0],
		[0.0, 0.0, 0.0, 0.005**2]])

	def set_field_center(self, data):
		self.center_of_field = config.vector3_to_numpy(data.position)

if __name__ == '__main__':
	DetermineVelocities()


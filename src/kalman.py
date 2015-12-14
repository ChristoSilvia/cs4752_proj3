#!/usr/bin/env python

# python
import numpy as np
import matplotlib.pyplot as plt

# ROS
import rospy
from geometry_msgs.msg import Point, PoseStamped
from cs4752_proj3.msg import BallPositionVelocity

rolling_threshold = 0.05
r_block
center_of_field = np.array([0.6, 0.0, -0.08])

class DetermineVelocities:
	def __init__(self):
		rospy.init_node('ball_position_velocity_kalman_filter')
		print("Initialized node DetermineVelocities")

		self.max_past_data_entries = 10
		self.state = None
		self.previous_time = None

		self.pub = rospy.Publisher('/ball_position_velocity', BallPositionVelocity, 
			queue_size=10) 
		rospy.Subscriber('/ball_pose_kinect', PoseStamped, self.handle_data)

        self.block_poses = [ np.array([ 0.6, 0.2, -0.08]),
            np.array([0.5, 0.3, -0.08]) ]
        self.last_min_block_distance = float('inf')


		rospy.spin()

	def handle_data(self, data):
		print("Recieved Data")
		t = data.header.stamp.secs + 1e-9*data.header.stamp.nsecs
		position = np.array([data.pose.position.x, data.pose.position.y])
		print("Ball Position: {0}".format(position))
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

			print("Update Matrix: {0}".format(update_matrix))

			offset = np.array([0.0, 0.0, 0.0, 0.0])

			state_to_measurement_matrix = np.array([
				[1.0, 0.0, 0.0, 0.0],
				[0.0, 1.0, 0.0, 0.0],
				[1.0, 0.0, -delta_t, 0.0],
				[0.0, 1.0, 0.0, -delta_t]])

            min_block_distance = min([ np.linalg.norm(block[:2] - position) for block in self.block_poses ])
            if (min_block_distance < block_radius) and (last_min_block_distance > block_radius):
                model_noise_matrix = np.array([
                    [0.02**2, 0.0, 0.0, 0.0],
                    [0.0, 0.02**2, 0.0, 0.0],
                    [0.0, 0.0, 0.5**2, 0.0],
                    [0.0, 0.0, 0.0, 0.5**2]])
            else:         
    			model_noise_matrix = np.array([
	    			[0.005**2, 0.0, 0.0, 0.0],
		    		[0.0, 0.005**2, 0.0, 0.0],
			    	[0.0, 0.0, 0.01**2, 0.0],
				    [0.0, 0.0, 0.0, 0.01**2]])

			# predict
			predicted_state = np.dot(update_matrix, self.state) + offset
			predicted_covariance = np.dot(update_matrix,np.dot(self.covariance, update_matrix.T)) + model_noise_matrix
			print("Predicted State: {0}".format(predicted_state))
			print("Predicted Covariance: {0}".format(predicted_covariance))
			
			# compute error
			measurement = np.empty(4)
			measurement[:2] = position
			measurement[2:] = self.previous_position
			print("Measured State: {0}".format(measurement))
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

	state_no_info_covariance = np.array(
		[[0.03 ** 2, 0.0, 0.0, 0.0],
		[0.0, 0.03 ** 2, 0.0, 0.0],
		[0.0, 0.0, 0.5 ** 2, 0.0],
		[0.0, 0.0, 0.0, 0.5 ** 2]])

	measurement_noise = np.array(
		[[0.01**2, 0.0, 0.0, 0.0],
		[0.0, 0.01**2, 0.0, 0.0],
		[0.0, 0.0, 0.01**2, 0.0],
		[0.0, 0.0, 0.0, 0.01**2]])

if __name__ == '__main__':
	DetermineVelocities()


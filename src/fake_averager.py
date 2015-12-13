#!/usr/bin/env python

# python
import numpy as np
import matplotlib.pyplot as plt

# ROS
import rospy
from geometry_msgs.msg import *
from cs4752_proj3.msg import BallPositionVelocity
from tf.transformations import *

class DetermineVelocities:
    def __init__(self):
		self.pub = rospy.Publisher('/ball_position_velocity', BallPositionVelocity,queue_size=10) 
		self.rviz_pub = rospy.Publisher('/ball_position_velocity_pose', PoseStamped,queue_size=10) 
		# self.pub = rospy.Publisher('/ball_position', Point ,queue_size=10) 
		self.init_time = rospy.Time.now().to_sec()
		rate = rospy.Rate(200)
		while not rospy.is_shutdown():
			t = rospy.Time.now().to_sec() - self.init_time
			# print t

			# v = 0.1
			# ry = .10
			# rx = .10
			# x = rx * np.cos(v*t)
			# y = ry * np.sin(v*t)
			# dx = rx * -np.sin(v*t)*v
			# dy = ry * np.cos(v*t)*v

			# x = -(2.0/3.0)*np.exp((t % 0.6) - 0.3)/(1.0 + np.exp((t % 0.6) - 0.3))
			# y = t % 0.6
			# dx = -(2.0/3.0)*np.exp((t % 0.6) - 0.3)/(1.0 + np.exp((t % 0.6) - 0.3))**2
			# dy = 1.0

			v = 0.5
			x = 0.0
			y = 0.0
			dx = np.cos(v*t)
			dy = np.sin(v*t)

			center_of_field = Point(0.5851, 0.0545, -0.0805)

			# position_velocity = BallPositionVelocity(
			# 	rospy.get_time(),
			# 	Point(0.5, 0.0, -0.8), # position
			# 	Point(0.0, -0.2, 0.0)) # velocity

			position = Point(center_of_field.x + x, center_of_field.y + y, center_of_field.z)
			velocity = Point(dx, dy, 0.0)
			position_velocity = BallPositionVelocity(rospy.get_time(), position, velocity)

			position_velocity_pose = PoseStamped()
			position_velocity_pose.header.frame_id = 'base'
			position_velocity_pose.header.stamp = rospy.Time.now()
			position_velocity_pose.pose.position = position
			q = quaternion_from_euler(0,0,0)
			position_velocity_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
			
			# print position_velocity.position
			# self.pub.publish(position_velocity)
			self.pub.publish(position_velocity)
			self.rviz_pub.publish(position_velocity_pose)
			# self.pub.publish(position_velocity.position)
			rate.sleep()
								


if __name__ == '__main__':
	rospy.init_node('fake_velocity')
	DetermineVelocities()


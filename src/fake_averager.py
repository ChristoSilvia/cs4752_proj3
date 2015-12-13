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

			ry = .10
			rx = .30
			x = rx * np.cos(t)
			y = ry * np.sin(t)
			dx = rx * -np.sin(t)
			dy = ry * np.cos(t)

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


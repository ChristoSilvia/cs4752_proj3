#!/usr/bin/env python

# python
import numpy as np
import matplotlib.pyplot as plt

# ROS
import rospy
from geometry_msgs.msg import Point, PoseStamped
from cs4752_proj3.msg import BallPositionVelocity

class DetermineVelocities:
    def __init__(self):
		self.pub = rospy.Publisher('/ball_position_velocity', BallPositionVelocity,queue_size=10) 

		while not rospy.is_shutdown():
			position_velocity = BallPositionVelocity(
				rospy.get_time(),
				Point(0.5, 0.0, -0.8), # position
				Point(0.0, -0.2, 0.0)) # velocity
			self.pub.publish(position_velocity)
								


if __name__ == '__main__':
	rospy.init_node('fake_velocity')
	DetermineVelocities()


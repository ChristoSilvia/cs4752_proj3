#!/usr/bin/env python

import rospy
import time

from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped

rospy.init_node('constant_publisher')

pub = rospy.Publisher('/test_pose', PoseStamped, queue_size=10)

frame = 0
rate = rospy.Rate(40)
while not rospy.is_shutdown():

	pose = PoseStamped()
	pose.pose.position.x = 0.5
	pose.pose.position.y = 0.5
	pose.pose.position.z = 0.0
	pose.pose.orientation.x = 1.0
	pose.pose.orientation.y = 0.0
	pose.pose.orientation.z = 0.0
	pose.pose.orientation.w = 0.0
	pose.header.seq = frame
	pose.header.stamp = rospy.Time()
	pose.header.frame_id = "what?"
	pub.publish(pose)

	frame += 1
	rate.sleep()


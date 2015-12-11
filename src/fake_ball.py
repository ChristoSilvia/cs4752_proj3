#!/usr/bin/env python

import rospy

from cs4752_proj3.msg import BlockerPosition

v = 0.06

rospy.init_node('things')
pub = rospy.Publisher('/blocker_position', BlockerPosition)

rate = rospy.Rate(30)

t_start = rospy.get_time()
x = 0.0
while (not rospy.is_shutdown()) and (x < 0.15):
    delta_t = rospy.get_time() - t_start
    x = v * delta_t 
    pub.publish(BlockerPosition(x))
    rate.sleep()



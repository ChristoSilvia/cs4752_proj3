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
        rospy.init_node('ball_position_velocity')
        print("Initialized node DetermineVelocities")

        self.max_past_data_entries = 10
        self.previous_position = None
        self.previous_time = None

        self.pub = rospy.Publisher('/ball_position_velocity', BallPositionVelocity, 
            queue_size=10) 
        rospy.Subscriber('/ball_pose_kinect', PoseStamped, self.handle_data)

        # plt.ion()
        # plt.show()

        # rate = rospy.Rate(10)
        # while True:
        #    if len(self.past_positions) is not 0:
        #        plt.clf()
        #        plt.scatter([ x[1] for x in self.past_positions],
        #                    [ x[2] for x in self.past_positions])
        #        plt.axis([-1,1,-1,1])
        #        plt.draw()
        #        rate.sleep()

        rospy.spin()

    def handle_data(self, data):
        #print("Recieved Data")
        t = data.header.stamp.secs + 1e-9*data.header.stamp.nsecs
        position = np.array([data.pose.position.x, data.pose.position.y])
        #print("At t={0} ball is at {1},{2}".format(t, data.pose.position.x, data.pose.position.y))

        if self.previous_position is None:
            self.previous_position = position
            self.previous_time = t
        else:
            v = (position - self.previous_position)/(t - self.previous_time)
            #print("Est Ball Velocity: {0},{1}".format(v[0], v[1]))
            self.previous_position = position
            self.previous_time = t
            self.pub.publish(BallPositionVelocity(t, data.pose.position, Point(v[0], v[1], 0)))

if __name__ == '__main__':
    DetermineVelocities()


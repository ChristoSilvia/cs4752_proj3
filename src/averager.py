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
        self.past_positions = []

        self.pub = rospy.Publisher('/ball_position_velocity', BallPositionVelocity, 
            queue_size=10) 
        rospy.Subscriber('/ball_pose', PoseStamped, self.handle_data)

        plt.ion()
        plt.show()

        rate = rospy.Rate(10)
        while True:
           if len(self.past_positions) is not 0:
               plt.clf()
               plt.scatter([ x[1] for x in self.past_positions],
                           [ x[2] for x in self.past_positions])
               plt.axis([-1,1,-1,1])
               plt.draw()
               rate.sleep()

        rospy.spin()

    def handle_data(self, data):
        print("Recieved Data")
        t = data.header.stamp.secs + 1e-9*data.header.stamp.nsecs
        data_point = np.array([t, data.pose.position.x, data.pose.position.y])
        print("At t={0} ball is at {1},{2}".format(t, data.pose.position.x, data.pose.position.y))

        if len(self.past_positions) == 0:
            self.past_positions = [ data_point for i in xrange(self.max_past_data_entries)]
        else:
            self.past_positions = self.past_positions[1:] + [data_point]
            coeffsx = np.polyfit([ self.past_positions[i][0]
                    for i in xrange(self.max_past_data_entries)],
                [ self.past_positions[i][1]
                    for i in xrange(self.max_past_data_entries)], 1)
            vx = coeffsx[0]
            coeffsy = np.polyfit([ self.past_positions[i][0]
                    for i in xrange(self.max_past_data_entries)],
                [ self.past_positions[i][2] 
                    for i in xrange(self.max_past_data_entries)], 1)
            vy = coeffsy[0]
            self.pub.publish(BallPositionVelocity(t, data.pose.position, Point(vx, vy, 0.0)))
            print("Moving with speed {0},{1}".format(vx, vy))



if __name__ == '__main__':
    DetermineVelocities()


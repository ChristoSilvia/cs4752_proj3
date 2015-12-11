#!/usr/bin/env python

import numpy as np

import rospy

from cs4752_proj3.msg import BlockerPosition, BallPositionVelocity


class ImpactLocationOnFarWall():
    def __init__(self, center_of_goal, limb_name):
        self.w = 0.6858
        self.l = 1.3843
        self.w_goal = 0.29845
        self.gripper_offset = 0.04
        self.center_of_goal = center_of_goal
        self.limb_name = limb_name

        self.y_velocity_cutoff = 1e-2

        rospy.init_node('impact_location_on_far_wall')
        print("Initialized node 'impact_location_on_far_wall'")
        self.pub = rospy.Publisher('/blocker_position', BlockerPosition, queue_size=10)

        rospy.Subscriber('/ball_position_velocity', 
                        BallPositionVelocity, 
                        self.handle_position_velocity_info)

        rospy.spin()


    def handle_position_velocity_info(self, data):
        print("\nRecieved data\n------------\n")
        if self.limb_name == "left"
            x = -(data.position.x - self.center_of_goal[0])
            y = -(data.position.y - self.center_of_goal[1])
        else:
            x = data.position.x - self.center_of_goal[0])
            y = data.position.y - self.center_of_goal[1])
            

        print("Ball Position: ({0},{1})".format(x,y))
        if np.abs(data.velocity.y) > self.y_velocity_cutoff:
            if self.limb_name == "left"
                tan_theta = (-data.velocity.x) / (data.velocity.y)
            else:
                tan_theta = ( data.velocity.x) / (-data.velocity.y)
            print("Ball Angle: {0}".format(tan_theta))
            x_blocker = self.target_position(x, y, tan_theta)
        else:
            print("Ball is Stationary")
            x_blocker = 0.0
            print("")

        self.pub.publish(BlockerPosition(x_blocker))
        print("Blocker x target: {0}".format(x_blocker))

    def impact_location_on_far_wall(self, x, y, tan_theta):
        unwalled_impact_location = x + y * tan_theta
        print("Unwalled impact location: {0}".format(unwalled_impact_location))
		
        impact_location_sign = np.sign(unwalled_impact_location)
		
        limited_impact_location = np.abs(unwalled_impact_location) % 2*self.w

        if limited_impact_location < 0.5*self.w:
            return impact_location_sign * limited_impact_location
        elif limited_impact_location < 1.5*self.w:
            return impact_location_sign * ( self.w - limited_impact_location)
        else:
            return impact_location_sign * ( limited_impact_location - 2.0*self.w)

    def target_position(self, x, y, tan_theta):
        impact_location = self.impact_location_on_far_wall(x, y, tan_theta)
        return np.clip(-0.5*self.w_goal + self.gripper_offset, 
                        0.5*self.w_goal - self.gripper_offset, 
                        impact_location)

if __name__ == '__main__':
    ImpactLocationOnFarWall(np.array([0.5853055, 0.7119757, -0.04525]), "left")

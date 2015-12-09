#!/usr/bin/env python

# python
import numpy as np


# ROS
import rospy

# Baxter
import baxter_interface

# baxter kinematics
from baxter_pykdl import baxter_kinematics

class Rotater:
    def __init__(self, limb_name):
        
        self.limb_name = limb_name
        self.limb = baxter_interface.Limb(limb_name)
        self.limb_kin = baxter_kinematics(limb_name)

        current_orientation = self.limb.endpoint_pose()['orientation']
        q_0 = np.array([current_orientation.x, 
                        current_orientation.y,
                        current_orientation.z,
                        current_orientation.w])

        q_1_unnormalized = np.array([0,0,0,1])
        if np.dot(q_0, q_1_unnormalized) < 0:
            q_1 = - q_1_unnormalized
        else:
            q_1 = q_1_unnormalized

        q = np.array([ q_0[3]*q_1[3] - q_0[0]*q_1[0] - q_0[1]*q_1[1] - q_0[2]*q_1[2],
                       q_0[0]*q_1[3] + q_0[3]*q_1[0] - q_0[1]*q_1[2] + q_0[2]*q_1[1],
                       q_0[1]*q_1[3] + q_0[3]*q_1[1] - q_0[0]*q_1[2] + q_0[2]*q_1[0],
                       q_0[2]*q_1[3] + q_0[3]*q_1[2] - q_0[1]*q_1[0] + q_0[0]*q_1[1]])
#        q = Quaternion(q_0.w*q_1.w - q_0
        theta = 2*np.arccos(q[0])
        k = q[1:]/np.sin(0.5*theta)

        print(theta)
        print(k)
        print(np.linalg.norm(k))

        print(q_0)

if __name__ == '__main__':
    rospy.init_node('rotater')
    baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()
    Rotater('right')

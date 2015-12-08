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

        print(current_orientation)

if __name__ == '__main__':
    rospy.init_node('rotater')
    baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()
    Rotater('right')

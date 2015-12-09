#!/usr/bin/env python

import rospy
import baxter_interface


class HandFrame():
    def __init__(self, limb_name):
        self.limb = baxter_interface.Limb(limb_name)
        
        while True:
           joint_values = self.limb.joint_angles()
           end_effector_pose = self.limb.endpoint_pose()
           end_effector_position = end_effector_pose.position
           end_effector_orientation = end_effector_pose.orientation

if __name__ == '__main__':
    rospy.init_node('basic')
    baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()
    HandFrame('right')

#!/usr/bin/env python

import sys
import rospy
import baxter_interface


class HandFrame():
    def __init__(self, limb_name):
        self.limb = baxter_interface.Limb(limb_name)
       
        rate = rospy.Rate(4)
 
        while not rospy.is_shutdown():
           joint_values = self.limb.joint_angles()
           end_effector_pose = self.limb.endpoint_pose()
           end_effector_position = end_effector_pose['position']
           end_effector_orientation = end_effector_pose['orientation']
           rospy.loginfo("------------------")
           rospy.loginfo("Arm {0}".format(limb_name))
           rospy.loginfo("Joint Values:")
           rospy.loginfo(joint_values)
           rospy.loginfo("EE Position")
           rospy.loginfo(end_effector_position)
           rospy.loginfo("EE Orientation")
           rospy.loginfo(end_effector_orientation)
           rospy.loginfo("------------------")
           rate.sleep()

if __name__ == '__main__':
    rospy.init_node('basic')
    baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()
    limb_name = sys.argv[1]
    assert (limb_name == "left" or limb_name == "right")
    print("Initializing constant shot on limb {0}".format(limb_name))
    HandFrame(limb_name)

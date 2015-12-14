#!/usr/bin/env python

import rospy

import baxter_interface

grabbing_joint_angles
intermediate_joint_angles
throwing_joint_angles

class Blind:
    def __init__(self, limb_name):
        self.limb_name = limb_name
        self.limb = baxter_interface.Limb(limb_name)
        self.gripper = baxter_interface.Gripper(limb_name)

        # calibration
        self.gripper.calibrate(block=True, timeout=5.0)

        while not rospy.is_shutdown():
            self.gripper.close(block=True, timeout=3.0)

            if gripper.missed():
                self.gripper.open(block=True, timeout=3.0)
                continue





if __name__ == '__main__':
    rospy.init_node('blind')
    print("Initialized node 'blind'")
    baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()

    try:
        limb_name = rospy.get_param("limb")
    except:
        if limb_name is None:
            limb_name = sys.argv[1]
    
    assert (limb_name == "left" or limb_name == "right")
    print("Initializing Blocker for {0} arm".format(limb_name))
    # POSITION OF GOAL

    # starts with gripper open, in a position to grab

    if limb_name == "left":
        Blind(limb_name, left_goal)
    else:
        Blind(limb_name, right_goal) 
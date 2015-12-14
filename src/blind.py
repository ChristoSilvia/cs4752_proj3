#!/usr/bin/env python

import numpy as np
import cv2
from cv_bridge import CvBridge

import rospy

from cs4752_proj3.srv import Action
from game_server.srv import *
from game_server.msg import *

import baxter_interface

class Blind:
    def __init__(self, limb_name):
        self.current_phase = 0
        self.game_state = rospy.Subscriber('/game_server/game_state', GameState, self.game_state_callback)

        self.limb_name = limb_name
        self.limb = baxter_interface.Limb(limb_name)
        self.gripper = baxter_interface.Gripper(limb_name)

        self.throw = rospy.ServiceProxy("throw", Action)

        # Phase 1
        self.gripper.calibrate(block=True, timeout=5.0)

        prompt = "Press Enter when Arm is in Pickup Position"
        cmd = raw_input(prompt)
        grabbing_joint_angles = self.limb.joint_angles()

        prompt = "Press Enter when Arm is in Intermediate Position"
        cmd = raw_input(prompt)
        intermediate_joint_angles = self.limb.joint_angles()

        prompt = "Press Enter to Continue"
        cmd = raw_input(prompt)

        while not rospy.is_shutdown() and self.current_phase == 1:
            pass

        # Phase 2

        self.gripper.open(block=False)

        self.limb.move_to_joint_positions(intermediate_joint_angles, timeout=10.0, threshold=0.05)

        self.limb.move_to_joint_positions(grabbing_joint_angles, timeout=2.0, threshold=0.02)

        while not rospy.is_shutdown() and self.current_phase == 2:
            pass

        while not rospy.is_shutdown() and self.current_phase == 3:
            self.limb.move_to_joint_angles(grabbing_joint_angles, timeout=2.0, threshold=0.02)
            self.gripper.close(block=True, timeout=3.0)

            if gripper.missed():
                self.gripper.open(block=True, timeout=3.0)
                rospy.sleep(1.0)
                continue

            self.limb.move_to_joint_positions(intermediate_joint_angles, timeout=5.0, threshold=0.05)
            
            self.throw()

            self.gripper.open(block=False)

            self.limb.move_to_joint_positions(intermediate_joint_angles, timeout=10.0, threshold=0.05)
            
    def game_state_callback(self, args):
        self.current_phase = args.current_phase


if __name__ == '__main__':
    rospy.init_node('blind')
    print("Initialized node 'blind'")
    baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()

    game_server = rospy.ServiceProxy('/game_server/init', Init)

    bridge = CvBridge()    
    img = cv2.imread('../ros_ws/src/cs4752_proj3/img/smith.jpg')
    logo = bridge.cv2_to_imgmsg(img, encoding='bgr8')

    limb_name = game_server('zic', logo).arm
    rospy.set_param('limb', limb_name)

    print("Got Limb Name: {0}".format(limb_name))
    
    Blind(limb_name)


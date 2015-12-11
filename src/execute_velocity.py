#!/usr/bin/env python

from config import *
import numpy as np
import rospy
import baxter_interface
import baxter_pykdl
from geometry_msgs.msg import Vector3

class Translator:
    def __init__(self, limb_name):
        self.limb_name = limb_name
        self.limb = baxter_interface.Limb(limb_name)
        self.limb_kin = baxter_pykdl.baxter_kinematics(limb_name)

        rospy.Subscriber('/execute_vel', Vector3, self.ex)
        print("subscription recorded")
        rospy.spin()

    def ex(self, vvector3):
        twist = np.empty(6)
        twist[:3] = vector3_to_numpy(vvector3)
        twist[3:] = np.zeros(3)

        jacobian = self.limb_kin.jacobian()
        jacobian_pinv = np.linalg.pinv(jacobian)

        joint_velocities = np.dot(jacobian_pinv, twist)

        joint_vel = numpy_to_joint_dict(self.limb_name, np.squeeze(np.asarray(joint_velocities)))

        self.limb.set_joint_velocities(joint_vel)

if __name__ == '__main__':
    rospy.init_node('execute_velocity')
    baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()

    limb_name = rospy.get_param('limb')
    Translator(limb_name)    

#!/usr/bin/env python
import numpy as np
from scipy.spatial import KDTree
from scipy.interpolate import PiecewisePolynomial
from scipy.optimize import minimize

import matplotlib.pyplot as plt

import rospy
from config import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from cs4752_proj2.srv import *
import baxter_interface
from baxter_interface import *
from baxter_core_msgs.msg import * #(SolvePositionIK, SolvePositionIKRequest)
from baxter_core_msgs.srv import *
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
from tf.transformations import *

MOVE_WAIT = 0.1
limb = "left"


def loginfo(logstring):
    rospy.loginfo("Controller: {0}".format(logstring))

def test():
    rospy.init_node('test')
    loginfo("Initialized node Controller")

    global move_robot
    joint_action_server = createServiceProxy("move_end_effector_trajectory",JointAction,limb)
    parameter_server = createServiceProxy("set_parameters",SetParameters,limb)

    T_max = 4.0
    L = 0.2
    position = position_server().position
    joint_action_server([0, T_max, T_max*2], [position, Vector3(position.x, position.y + L, position.z), Vector3(position.x + L/2, position.y + L, position.z)], [Vector3(0,v,0),Vector3(0,v,0),Vector3(v,0,0)])


if __name__ == '__main__':
    test()

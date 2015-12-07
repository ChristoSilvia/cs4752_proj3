#!/usr/bin/env python
import numpy as np
from scipy.spatial import KDTree
from scipy.interpolate import PiecewisePolynomial


import rospy
from geometry_msgs.msg import Vector3
from cs4752_proj2.srv import *
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
from tf.transformations import *


def loginfo(logstring):
    rospy.loginfo("Controller: {0}".format(logstring))

def test():
    rospy.init_node('test')
    loginfo("Initialized node Controller")

    rospy.wait_for_service("/move_end_effector_trajectory")
    joint_action_server = rospy.ServiceProxy("/move_end_effector_trajectory", JointAction)
    tool_trajectory = rospy.ServiceProxy("/move_tool_trajectory", JointAction)
    loginfo("Initialized Joint Action Server Proxy")
    rospy.wait_for_service("/end_effector_position")
    position_server = rospy.ServiceProxy("/end_effector_position", EndEffectorPosition)
    loginfo("Initialized position server proxy")

#    rospy.sleep(5.0)

    loginfo("Making position call")
    initial_position = position_server().position
    loginfo(initial_position)
    joint_action_server([5.0, 10.0],
                        [Vector3(initial_position.x + 0.1*np.random.rand(),
                                 initial_position.y + 0.1*np.random.rand(),
                                 initial_position.z + 0.1*np.random.rand()),
                         initial_position],
                        [Vector3(0.11, 0.0, 0.0),
                         Vector3(0.0, 0.0, 0.0)])
    loginfo(position_server().position.x - initial_position.x)      
    loginfo(position_server().position.y - initial_position.y)      
    loginfo(position_server().position.z - initial_position.z)      
     
#       joint_action_server([4.0, 8.0], 
#                           [Vector3(current_position.x+0.05, 
#                                    current_position.y-0.05, 
#                                    current_position.z-0.005),
#                            Vector3(current_position.x,
#                                    current_position.y-0.1,
#                                    current_position.z-0.01)],    
#                           [Vector3(0.0,-0.05,0.0), Vector3(0.05,0.0,0.0)])

if __name__ == '__main__':
    test()

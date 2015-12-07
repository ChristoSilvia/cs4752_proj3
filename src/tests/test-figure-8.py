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
    move_robot = rospy.ServiceProxy("/move_robot", MoveRobot)
    log_info("Initialized service proxy for /move_robot")
    
    rospy.wait_for_service("/move_end_effector_trajectory")
    joint_action_server = rospy.ServiceProxy("/move_end_effector_trajectory", JointAction)
    tool_trajectory = rospy.ServiceProxy("/move_tool_trajectory", JointAction)
    loginfo("Initialized Joint Action Server Proxy")
    rospy.wait_for_service("/end_effector_position")
    position_server = rospy.ServiceProxy("/end_effector_position", EndEffectorPosition)
    loginfo("Initialized position server proxy")
    parameter_server = rospy.ServiceProxy("/set_parameters", SetParameters)
    loginfo("Initialized parameter server")

    ns = "ExternalTools/"+limb+"/PositionKinematicsNode/IKService"
    try :
        rospy.loginfo("Initializing service proxy for /SolvePositionIK...")
        global iksvc
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        rospy.loginfo("Initialized service proxy for /SolvePositionIK...")
    except rospy.ServiceException, e:
        rospy.logerr("Service Call Failed: {0}".format(e))
    print "Ready to move robot."
        

    T_max = 20.0
    n_samples = 20
    T = np.linspace(0,T_max,n_samples)
    times = list(T)

    base_position = position_server().position
    

    def evaluate_parameters(params):
        loginfo(params)
        A = 0.03
        n_tests = 1
        parameter_server(params[0],params[1],params[2],params[3],params[4])
        Tau = (T/T_max)**2 * (4.0 - 4.0*(T/T_max) + (T/T_max)**2)
        Tauprime = 2*(T/(T_max**2))*(4.0 - 4.0*(T/T_max) + (T/T_max)**2) + (T/T_max)**2 * (-4.0/T_max + 2.0*(T/(T_max**2)))
        X = A * np.sin(0.5 * (4.0 * np.pi * Tau))
        Y = A * np.sin(4.0 * np.pi * Tau)
        Xprime = A * np.cos(0.5 * (4.0 * np.pi * Tau)) * 0.5 * 4.0 * np.pi * Tauprime
        Yprime = A * np.cos(4.0 * np.pi * Tau) * 4.0 * np.pi * Tauprime   

        # plt.plot(X,Y)
        # plt.show()
        # plt.plot(T,X)
        # plt.plot(T, Xprime)
        # plt.show()
        # plt.plot(T, Y)
        # plt.plot(T, Yprime)
        # plt.show()
 

        errors = np.empty(n_tests)
        for j in xrange(n_tests):
            # HomePose()
            # joint_action_server([0.0, 2.0], [position_server().position, base_position], [Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)])           
            initial_position = position_server().position
            positions = []
            velocities = []
            for i in xrange(len(T)):
                positions = positions + [Vector3(initial_position.x + X[i], initial_position.y + Y[i], initial_position.z)]
                velocities = velocities + [Vector3(Xprime[i], Yprime[i] , 0.0)]

            rospy.sleep(1.0)          
 
            joint_action_server(times, positions, velocities)
            final_position = position_server().position
            errors[j] = np.sqrt( (initial_position.x - final_position.x)**2 + (initial_position.y - final_position.y)**2 + (initial_position.z - final_position.z)**2)
            loginfo(errors[j])

            # joint_action_server([0.0,2.0], [initial_position, base_position], [Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)])
       
        return np.mean(errors)


    K_p = 1.5
    K_i = 0.72
    K_d = -0.0054
    guess_params = np.array([K_p, K_i, K_d, 0.05, 1.5])
    evaluate_parameters(guess_params)
    # HomePose()
    # sweet_params = minimize(evaluate_parameters, 
    #                        np.array([0.01, 0.01, 0.0001, 0.05, 2.0]),
    #                        method='Nelder-Mead')
    #loginfo("resultant parameters: {0}".format(sweet_params))

def HomePose() :
    rospy.loginfo("Going to Home Pose")
    homepose = Pose()
    homepose.position = Point(0.572578886689,0.281184911298,0.146191403844)
    homepose.orientation = Quaternion(0.140770659119,0.989645234506,0.0116543447684,0.0254972076605)
    # success = MoveToPose(homepose, False, False, False)
    global move_robot
    success = move_robot(MOVE_TO_POSE, "left", homepose)
    rospy.loginfo("Got to Home Pose : %r", success)

    
if __name__ == '__main__':
    test()

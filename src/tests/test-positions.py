#!/usr/bin/env python
import numpy as np
from scipy.spatial import KDTree
from scipy.interpolate import PiecewisePolynomial
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from config import *
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from cs4752_proj2.srv import *
from cs4752_proj2.msg import *
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
    rospy.loginfo("test: {0}".format(logstring))

class TestPositionControl():
    def __init__(self):
        rospy.init_node('test')
        loginfo("Initialized node test")

        rospy.wait_for_service("/move_robot")
        self.position_controller = rospy.ServiceProxy("/move_robot", MoveRobot)

        self.HomePose()

    def HomePose(self) :
        rospy.loginfo("Going to Home Pose")
        homepose = Pose()
        homepose.position = Point(0.572578886689,0.281184911298,0.146191403844)
        homepose.orientation = Quaternion(0.140770659119,0.989645234506,0.0116543447684,0.0254972076605)
        # success = MoveToPose(homepose, False, False, False)
        success = self.position_controller(MOVE_TO_POSE, "left", homepose)
        rospy.loginfo("Got to Home Pose : %r", success)



    # rospy.wait_for_service("/move_end_effector_trajectory")
    # joint_action_server = rospy.ServiceProxy("/move_end_effector_trajectory", JointAction)
    # tool_trajectory = rospy.ServiceProxy("/move_tool_trajectory", JointAction)
    # loginfo("Initialized Joint Action Server Proxy")
    # rospy.wait_for_service("/end_effector_position")
    # position_server = rospy.ServiceProxy("/end_effector_position", EndEffectorPosition)
    # loginfo("Initialized position server proxy")
    # parameter_server = rospy.ServiceProxy("/set_parameters", SetParameters)
    # loginfo("Initialized parameter server")

#     ns = "ExternalTools/"+limb+"/PositionKinematicsNode/IKService"
#     try :
#         rospy.loginfo("Initializing service proxy for /SolvePositionIK...")
#         global iksvc
#         iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
#         rospy.wait_for_service(ns, 5.0)
#         rospy.loginfo("Initialized service proxy for /SolvePositionIK...")
#     except rospy.ServiceException, e:
#         rospy.logerr("Service Call Failed: {0}".format(e))
#     print "Ready to move robot."
        

#     T_max = 20.0
#     n_samples = 20
#     T = np.linspace(0,T_max,n_samples)
#     times = list(T)

#     base_position = position_server().position
    

#     def evaluate_parameters(params):
#         loginfo(params)
#         A = 0.03
#         n_tests = 1
#         parameter_server(params[0],params[1],params[2],params[3],params[4])
#         Tau = (T/T_max)**2 * (4.0 - 4.0*(T/T_max) + (T/T_max)**2)
#         Tauprime = 2*(T/(T_max**2))*(4.0 - 4.0*(T/T_max) + (T/T_max)**2) + (T/T_max)**2 * (-4.0/T_max + 2.0*(T/(T_max**2)))
#         X = A * np.sin(0.5 * (4.0 * np.pi * Tau))
#         Y = A * np.sin(4.0 * np.pi * Tau)
#         Xprime = A * np.cos(0.5 * (4.0 * np.pi * Tau)) * 0.5 * 4.0 * np.pi * Tauprime
#         Yprime = A * np.cos(4.0 * np.pi * Tau) * 4.0 * np.pi * Tauprime   

#         # plt.plot(X,Y)
#         # plt.show()
#         # plt.plot(T,X)
#         # plt.plot(T, Xprime)
#         # plt.show()
#         # plt.plot(T, Y)
#         # plt.plot(T, Yprime)
#         # plt.show()
 

#         errors = np.empty(n_tests)
#         for j in xrange(n_tests):
#             # HomePose()
#             # joint_action_server([0.0, 2.0], [position_server().position, base_position], [Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)])           
#             initial_position = position_server().position
#             positions = []
#             velocities = []
#             for i in xrange(len(T)):
#                 positions = positions + [Vector3(initial_position.x + X[i], initial_position.y + Y[i], initial_position.z)]
#                 velocities = velocities + [Vector3(Xprime[i], Yprime[i] , 0.0)]

#             rospy.sleep(1.0)          
 
#             joint_action_server(times, positions, velocities)
#             final_position = position_server().position
#             errors[j] = np.sqrt( (initial_position.x - final_position.x)**2 + (initial_position.y - final_position.y)**2 + (initial_position.z - final_position.z)**2)
#             loginfo(errors[j])

#             # joint_action_server([0.0,2.0], [initial_position, base_position], [Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)])
       
#         return np.mean(errors)


#     K_p = 1.5
#     K_i = 0.72
#     K_d = -0.0054
#     guess_params = np.array([K_p, K_i, K_d, 0.05, 1.5])
#     evaluate_parameters(guess_params)
#     # HomePose()
#     # sweet_params = minimize(evaluate_parameters, 
#     #                        np.array([0.01, 0.01, 0.0001, 0.05, 2.0]),
#     #                        method='Nelder-Mead')
#     #loginfo("resultant parameters: {0}".format(sweet_params))

# def HomePose() :
#     rospy.loginfo("Going to Home Pose")
#     homepose = Pose()
#     homepose.position = Point(0.572578886689,0.181184911298,0.146191403844)
#     homepose.orientation = Quaternion(0.140770659119,0.989645234506,0.0116543447684,0.0254972076605)
#     # success = MoveToPose(homepose, False, False, False)
#     success = MoveToPose(homepose, False, False, False)
#     rospy.loginfo("Got to Home Pose : %r", success)

# def MoveToPose (pose, inter1=True, inter2=True, inter3=True) :
#     # global hand_pose
#     global MOVE_WAIT

#     # if inter1 :
#     #     b1 = MoveToIntermediatePose(hand_pose)
#     # if inter2 :
#     #     b2 = MoveToIntermediatePose(pose)
#     # if inter2 :
#     #     b3 = MoveToRightAbovePose(pose)

#     joint_solution = inverse_kinematics(pose)
#     if joint_solution != [] :
#         moveArm(joint_solution)
#         rospy.sleep(MOVE_WAIT)
#         return True
#     else :
#         rospy.logerr("FAILED MoveToPose")
#         return False

# def moveArm (joint_solution) :
#     global limb
#     arm = Limb(limb)
#     #while not rospy.is_shutdown():
#     arm.move_to_joint_positions(joint_solution)
#     rospy.sleep(0.01)

# #takes position in base frame of where hand is to go
# #calculates ik and moves limb to that location
# #returns 1 if successful and 0 if invalid solution
# def inverse_kinematics(ourpose) :
#     # given x,y,z will call ik for this position with identity quaternion
#     #in base frame
#     global limb
#     ikreq = SolvePositionIKRequest()

#     hdr = Header(stamp=rospy.Time.now(), frame_id='base')
#     poses = {
#         limb : PoseStamped(
#             header = hdr,
#             pose = ourpose
#         ),
#     }         
#     #getting ik of pose
     
#     ikreq.pose_stamp.append(poses[limb])

#     try :
#         ns = "ExternalTools/"+limb+"/PositionKinematicsNode/IKService"
#         rospy.wait_for_service(ns, 5.0)
#         resp = iksvc(ikreq)
#     except (rospy.ServiceException, rospy.ROSException), e:
#         rospy.logerr("Service call failed: %s" % (e,))
#         return []
#     if (resp.isValid[0]):
#         print("SUCCESS - Valid Joint Solution Found:")
#         limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
#         #print limb_joints
#         return limb_joints
#     else :
#         rospy.logerr("Invalid pose")
#         return []
    
    
if __name__ == '__main__':
    TestPositionControl()

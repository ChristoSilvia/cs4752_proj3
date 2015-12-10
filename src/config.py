#!/usr/bin/env python
# Team: zic; Names: Zach Vinegar, Isaac Qureshi, Chris Silvia
import rospy
import numpy as np
from geometry_msgs.msg import *

joint_names = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']

CLOSE_GRIPPER = 0
OPEN_GRIPPER = 1
MOVE_TO_POSE = 2
MOVE_TO_POS = 3
MOVE_TO_POSE_INTERMEDIATE = 4


BLOCK = 10
GRAB = 13
THROW = 14
CHECK_BLOCKS = 15
MOVE_BLOCKS = 16



def createServiceProxy(service,srv_type,limb):
	name = ""
	if limb == "":
		name = "/%s" % (service)
	else:
		name = "/%s/%s" % (limb,service)
	rospy.wait_for_service(name)
	rospy.loginfo("Initialized service proxy for %s" % name)
	return rospy.ServiceProxy(name, srv_type)

def createService(service,srv_type,callback,limb):
	name = ""
	if limb == "":
		name = "/%s" % (service)
	else:
		name = "/%s/%s" % (limb,service)
	rospy.loginfo("Initialized service for %s" % name)
	return rospy.Service(name, srv_type, callback)

def vector3_to_numpy(v):
	return np.array([v.x, v.y, v.z])

def quaternion_to_numpy(q):
	return np.array([q.x, q.y, q.z, q.w])

def numpy_to_vector3(n):
	return Vector3(n[0],n[1],n[2])

def numpy_to_joint_dict(limb, data):
	print data
	joint_dict = {}
	for i in range(0,len(joint_names)):
		joint_dict[ limb + "_" + joint_names[i] 
		] = data[i]
	return joint_dict

def quaternion_to_numpy(q):
	return np.array([q.x, q.y, q.z, q.w])

def invert_unit_quaternion(q):
        return np.array([-q[0], -q[1], -q[2], q[3]])

def multiply_quaternion(q_0, q_1):
	return np.array([ q_0[0]*q_1[3] + q_0[3]*q_1[0] - q_0[1]*q_1[2] + q_0[2]*q_1[1],
                       q_0[1]*q_1[3] + q_0[3]*q_1[1] - q_0[0]*q_1[2] + q_0[2]*q_1[0],
                       q_0[2]*q_1[3] + q_0[3]*q_1[2] - q_0[1]*q_1[0] + q_0[0]*q_1[1],
                       q_0[3]*q_1[3] - q_0[0]*q_1[0] - q_0[1]*q_1[1] - q_0[2]*q_1[2] ])

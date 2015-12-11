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

def mirror_left_arm_joints(new_pos):

	joint_limits = {
		"s0": [-2.461, 0.890],
		"s1": [-2.147, 1.047],
		"e0": [-3.028, 3.028],
		"e1": [-0.052, 2.618],
		"w0": [-3.059, 3.059],
		"w1": [-1.571, 2.094],
		"w2": [-3.059, 3.059]
	}

	joint_middle = {
		's0': 0.0, 
		's1': -0.55, 
		'e0': 0.0,
		'e1': 1.283, 
		'w0': 0.0, 
		'w1': 0.2615, 
		'w2': 0.0
	}
	for key in new_pos:
				limbless_key = key[-2:]
				if limbless_key != 'e1' and limbless_key != 's1' and limbless_key != 'w1':
					new_pos[key] = joint_middle[limbless_key] - (new_pos[key] - joint_middle[limbless_key])

	return new_pos
#!/usr/bin/env python
# Team: zic; Names: Zach Vinegar, Isaac Qureshi, Chris Silvia
import rospy

CLOSE_GRIPPER = 0
OPEN_GRIPPER = 1
MOVE_TO_POSE = 2
MOVE_TO_POS = 3
MOVE_TO_POSE_INTERMEDIATE = 4


BLOCK = 10
# INTERCEPT = 11
SEARCH = 12
GRAB = 13
THROW = 14
CHECK_BLOCKS = 15
MOVE_BLOCKS = 16

indexes = {
	"BOTTOM_CORNER": 0,
	"BOTTOM_B_NEAR_GOAL": 1,
	"BOTTOM_B_NEAR_MIDDLE": 2,
	"BOTTOM_MIDDLE": 3,
	"TOP_CORNER": 4,
	"TOP_B_NEAR_GOAL": 5,
	"TOP_B_NEAR_MIDDLE": 6,
	"TOP_MIDDLE": 7,
	"GOAL": 8,
	"BALL_START": 9,
	"BLOCK_START": 10
}

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

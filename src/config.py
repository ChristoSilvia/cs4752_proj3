#!/usr/bin/env python
# Team: zic; Names: Zach Vinegar, Isaac Qureshi, Chris Silvia
import rospy

CLOSE_GRIPPER = 0
OPEN_GRIPPER = 1
MOVE_TO_POSE = 2
MOVE_TO_POS = 3
MOVE_TO_POSE_INTERMEDIATE = 4

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

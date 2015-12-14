#!/usr/bin/env python
# Team: zic; Names: Zach Vinegar, Isaac Qureshi, Chris Silvia
import rospy
import numpy as np
from scipy.spatial import KDTree
from scipy.interpolate import PiecewisePolynomial
from matplotlib import pyplot as plt
from cs4752_proj3.msg import *
from cs4752_proj3.srv import *
from config import *
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker, MarkerArray
import baxter_interface
from baxter_interface import *
from baxter_pykdl import baxter_kinematics
import tf
from tf.transformations import *
from copy import deepcopy
import cv2



def loginfo(logstring):
    rospy.loginfo("Block_mover: {0}".format(logstring))

class block_mover() :
    def __init__(self):
        rospy.init_node('block_mover')
        loginfo("Initialized node Block_mover")

        baxter_interface.RobotEnable(CHECK_VERSION).enable()

        self.limb_name = rospy.get_param("limb")
        
        self.move_robot = createServiceProxy("move_robot", MoveRobot, "")

        self.get_desired_block_poses = createServiceProxy("get_desired_block_poses", BlockPoses, "")

        init_blocks = createService("init_blocks", BlockPoses, self.setDesiredBlockPoses, "")
        self.arm = baxter_interface.Limb(self.limb_name)

        self.block_size = .045
        
        rospy.spin()

    def getInitBlockPoses(self):
        rospy.loginfo("Initializing block positions")
        self.initial_pose = Pose()
        pos = np.array(self.arm.endpoint_pose()['position'])
        ori = np.array(self.arm.endpoint_pose()['orientation'])
        self.initial_pose.position = Point(pos[0],pos[1],pos[2])
        self.initial_pose.orientation = Quaternion(ori[0],ori[1],ori[2],ori[3])
        # print self.initial_pose

        self.table_z = self.initial_pose.position.z - ((self.num_blocks -1) * self.block_size)
        block_poses = []
        for i in range(0,self.num_blocks) :
            bp = deepcopy(self.initial_pose)
            bp.position.z -= i * self.block_size
            block_poses.append(bp)
        # print self.block_poses
        return block_poses

    def setDesiredBlockPoses(self, req):

        self.num_blocks = req.num_blocks

        self.init_block_poses = self.getInitBlockPoses()

        self.desired_block_poses = self.get_desired_block_poses(self.num_blocks).poses

        for desired_block_pose in self.desired_block_poses:
            desired_block_pose.orientation = deepcopy(self.initial_pose.orientation)

        # for i in range(0,self.num_blocks):
        #     if i != 0:
        #         self.move_robot(MOVE_TO_POSE_INTERMEDIATE, self.limb_name, self.init_block_poses[i])

        #     self.move_robot(CLOSE_GRIPPER, self.limb_name, Pose())

        #     self.move_robot(MOVE_TO_POSE_INTERMEDIATE, self.limb_name, self.desired_block_poses[i])

        #     self.move_robot(OPEN_GRIPPER, self.limb_name, Pose())

        return BlockPosesResponse(self.desired_block_poses)


if __name__ == '__main__':
    ct = block_mover()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

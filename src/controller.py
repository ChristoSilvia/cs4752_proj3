#!/usr/bin/env python
# Team: zic; Names: Zach Vinegar, Isaac Qureshi, Chris Silvia
import rospy
import numpy as np
from scipy.spatial import KDTree
from scipy.interpolate import PiecewisePolynomial
from matplotlib import pyplot as plt
from cs4752_proj3.msg import *
from cs4752_proj3.srv import *
from game_server.msg import *
from game_server.srv import *
from config import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from visualization_msgs.msg import Marker, MarkerArray
import baxter_interface
from baxter_interface import *
from baxter_pykdl import baxter_kinematics
import tf
from tf.transformations import *
from copy import deepcopy
import cv2


def loginfo(logstring):
    rospy.loginfo("Controller: {0}".format(logstring))

class controller() :
    def __init__(self, limb):
        rospy.init_node('controller')
        loginfo("Initialized node Controller")

        baxter_interface.RobotEnable(CHECK_VERSION).enable()

        # self.game_init = createServiceProxy("game server/init", Init, "")
        # self.limb_name = self.game_init("ZIC",None)
        # rospy.set_param("limb", self.limb_name)
        self.game_state_sub = rospy.Subscriber("/game_server/game_state", GameState, self.game_state_callback, queue_size=1)

        self.limb_name = rospy.get_param("limb")
        self.limb_name = 'left'
        self.num_blocks = rospy.get_param("num_blocks")

        self.move_robot = createServiceProxy("move_robot", MoveRobot, "")

        # self.block = createServiceProxy('block', Action, "")
        # self.grasp = createServiceProxy("grasp", Action, "")

        # self.get_block_poses = createServiceProxy("get_block_poses", BlockPoses, "")

        self.block_size = .045
        self.arm = baxter_interface.Limb(self.limb_name)
        # self.PHASE = rospy.get_param("phase")
        self.PHASE = 2

        self.rate = rospy.Rate(1)

        self.playing_field = {}
        loginfo("Starting GameLoop on %s arm" % self.limb_name)
        self.GameLoop()

    def game_state_callback(self, msg):
        pass

    def PHASE1(self):
        loginfo("PHASE: 1")
        
        # self.calibrate_kinect = createServiceProxy("calibrate_kinect", CalibrateKinect, "")

        def CALIBRATE():
            loginfo("Calibrating Playing Field and Kinect Frame")
            calibration_points = ["BOTTOM_MIDDLE", "BOTTOM_CORNER", "TOP_MIDDLE", "TOP_CORNER"]
            
            res = self.calibrate_kinect(calibration_points)

            loginfo("Finished Calibrating Kinect: %r" % res)

        CALIBRATE()

    def PHASE2(self):

        self.init_blocks = createServiceProxy("init_blocks", BlockPoses, "")

        loginfo("PHASE: 2")

        def INIT_BLOCKS():
            loginfo("Initializing Blocks")
            
            res = self.init_blocks(self.num_blocks)
            print res

            loginfo("Finished Initializing Blocks")

        INIT_BLOCKS()

    def PHASE3(self):

        def BLOCK():
            req = ActionRequest()
            req.action = BLOCK
            req.limb = self.limb_name

            block_res = self.block(req)
            return block_res.success

        def GRAB():
            req = ActionRequest()
            req.action = GRAB
            req.limb = self.limb_name
            req.arg = 'pink'
            
            grasp_res = self.grasp(req)
            return grasp_res.success

        def CHECK_BLOCKS():
            # block_poses = self.get_block_poses(self.num_blocks)
            return True
            # return False

        def MOVE_BLOCKS():
            pass

        def THROW():
            req = ActionRequest()
            req.action = THROW
            req.limb = self.limb_name
            
            throw_res = self.throw(req)
            return throw_res.success

    
        loginfo("PHASE: 3")
        self.MODE = BLOCK

        self.playing = True
        
        while not rospy.is_shutdown() and self.playing:

            if self.MODE == BLOCK :
                loginfo("MODE: BLOCK")
                # returns true when its sure the ball not going in the goal and still on our side,
                # false if on other side
                blocked = BLOCK()
                if blocked:
                    self.MODE = GRAB

                
            elif self.MODE == GRAB :
                loginfo("MODE: GRAB")
                # returns true if grabbing was successful, false if the ball goes on the other side
                grabbed = GRAB()
                if grabbed:
                    self.MODE = CHECK_BLOCKS
                else:
                    self.MODE = BLOCK

                    
            elif self.MODE == CHECK_BLOCKS :
                loginfo("MODE: CHECK_BLOCKS")
                # moves the arm out of the way and then uses vision to find the blocks
                # returns true if the blocks are in the right place, false if they need to be moved
                blocks_ok = CHECK_BLOCKS()
                if blocks_ok:
                    self.MODE = THROW
                else:
                    self.MODE = MOVE_BLOCKS

                
            elif self.MODE == MOVE_BLOCKS :
                loginfo("MODE: MOVE_BLOCKS")
                # returns after the the blocks have been moved
                MOVE_BLOCKS()
                self.MODE = GRAB

                
            elif self.MODE == THROW :
                loginfo("MODE: THROW")
                # returns after the throw
                THROW()
                self.MODE = BLOCK

            self.rate.sleep()

    def GameLoop(self):
        if self.PHASE == 1 :
            # returns after calibration
            self.PHASE1()
            self.PHASE = 2

        elif self.PHASE == 2 :
            # returns after the the blocks have been moved
            "got here"
            self.PHASE2()
            self.PHASE = 3

        elif self.PHASE == 3 :
            self.PHASE3()

    def get_tool_pos(self):
        tool_vec = self.position_server().position
        # tool_vec = self.tool_position_server().position
        tool = np.array([tool_vec.x,tool_vec.y,tool_vec.z])
        return tool


if __name__ == '__main__':
    limb = 'left'
    ct = controller(limb)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

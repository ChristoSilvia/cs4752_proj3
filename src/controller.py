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
from cv_bridge import CvBridge, CvBridgeError


def loginfo(logstring):
    rospy.loginfo("Controller: {0}".format(logstring))

class controller() :
    def __init__(self):
        rospy.init_node('controller')
        loginfo("Initialized node Controller")


        self.bridge = CvBridge()    
        img = cv2.imread('../ros_ws/src/cs4752_proj3/img/smith.jpg')
        self.logo = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')

        # self.game_init = createServiceProxy("game server/init", Init, "")
        # self.limb_name = self.game_init("ZIC",self.logo)
        # rospy.set_param("limb", self.limb_name)
        # print "Initialized game"

        # try:
        #     init_robot = rospy.ServiceProxy('/game_server/init', Init)
        #     self.limb_name = init_robot('ZIC',self.logo)
        #     rospy.set_param("limb", self.limb_name)
        #     if self.limb_name == "none":
        #         print "You were not given an arm!"
        #     else:
        #         print "You have been given the %s arm!" % self.limb_name
        # except rospy.ServiceException, e:
        #     print "Service call failed: %s"%e
        self.limb_name = rospy.get_param("limb")

        # print "Initialized game"

        baxter_interface.RobotEnable(CHECK_VERSION).enable()
        # limb_srv = 

        self.game_state_sub = rospy.Subscriber("/game_server/game_state", GameState, self.game_state_callback, queue_size=1)
        self.ball_pos_vel = rospy.Subscriber("ball_position_velocity", BallPositionVelocity, self.ball_pos_vel_callback, queue_size=1)

        self.num_blocks = rospy.get_param("num_blocks")

        self.move_robot = createServiceProxy("move_robot", MoveRobot, "")

        self.block = createServiceProxy('block', Action, "")
        self.grasp = createServiceProxy("grasp", Action, "")

        # self.get_block_poses = createServiceProxy("get_block_poses", BlockPoses, "")

        # self.goal_center_joint_angles_pub = rospy.Publisher('/goal_center_joint_angles', JointAngles, queue_size=10)
        self.goal_center_pose_pub = rospy.Publisher('/goal_center_pose', Pose, queue_size=10)
        self.field_center_pose_pub = rospy.Publisher('/field_center_pose', Pose, queue_size=10)

        self.block_size = .045
        self.arm = baxter_interface.Limb(self.limb_name)
        # self.PHASE = rospy.get_param("phase")
        self.PHASE = 1
        # self.ball_on_side = rospy.get_param("ball_start")
        self.ball_on_side = 'left'

        self.rate = rospy.Rate(1)

        self.playing_field = {}
        loginfo("Starting GameLoop on %s arm" % self.limb_name)
        self.GameLoop()

    def game_state_callback(self, msg):
        self.PHASE = msg.current_phase

    def ball_pos_vel_callback(self, msg):
        self.ball_pos = msg.position
        # self.prev_on_side = self.ball_on_side
        # if msg.position.y > 0:
        #     self.ball_on_side = 'left'
        #     # print "ball on left side"
        # if msg.position.y < 0:
        #     self.ball_on_side = 'right'
        #     # print "ball on right side"

        # if self.ball_on_side != self.prev_on_side:
        #     print "Ball changed sides"

    

    def PHASE1(self):
        loginfo("PHASE: 1")
        
        # self.calibrate_kinect = createServiceProxy("calibrate_kinect", CalibrateKinect, "")

        def CALIBRATE():

            prompt = "Press Enter when Arm is at the center goal position"
            cmd = raw_input(prompt)
            self.goal_joints = self.arm.joint_angles()
            # joint_angles = list(joint_dict_to_numpy(self.arm.joint_angles()))
            # self.goal_center_joint_angles_pub.publish(joint_angles)
            self.goal_center_pose = get_current_pose(self.arm)
            self.goal_center_pose_pub.publish(get_current_pose(self.arm))

            prompt = "Press Enter when Arm is at the field center"
            cmd = raw_input(prompt)
            self.field_center_pose_pub.publish(get_current_pose(self.arm))

            prompt = "Press Enter when Arm is at the above ball start position"
            cmd = raw_input(prompt)
            self.above_ball_start = get_current_pose(self.arm)

            prompt = "Press Enter when Arm is out of the way for block pose selection"
            cmd = raw_input(prompt)

            # loginfo("Calibrating Playing Field and Kinect Frame")
            # calibration_points = ["BOTTOM_MIDDLE", "BOTTOM_CORNER", "TOP_MIDDLE", "TOP_CORNER"]
            
            # res = self.calibrate_kinect(calibration_points)

            # loginfo("Finished Calibrating Kinect: %r" % res)

        CALIBRATE()

    def PHASE2(self):

        self.init_blocks = createServiceProxy("init_blocks", BlockPoses, "")

        loginfo("PHASE: 2")

        def INIT_BLOCKS():
            loginfo("Initializing Blocks")
            
            self.move_robot(OPEN_GRIPPER, self.limb_name, Pose())
            
            res = self.init_blocks(self.num_blocks)
            print res

            loginfo("Finished Initializing Blocks")

        INIT_BLOCKS()

    def PHASE3(self):

        def _BLOCK():
            req = ActionRequest()
            req.action = BLOCK
            req.limb = self.limb_name

            # self.arm.set_joint_position_speed(.1)
            
            self.move_robot(MOVE_TO_POSE, self.limb_name, self.goal_center_pose)
            block_res = self.block(req)
            return block_res.success

        def _GRAB():
            req = ActionRequest()
            req.action = GRAB
            req.limb = self.limb_name
            req.arg = 'pink'
            
            self.move_robot(MOVE_TO_POSE, self.limb_name, self.above_ball_start)
            grasp_res = self.grasp(req)
            return grasp_res.success

        # def CHECK_BLOCKS():
        #     # block_poses = self.get_block_poses(self.num_blocks)
        #     return True
        #     # return False

        # def MOVE_BLOCKS():
        #     pass

        def _THROW():
            req = ActionRequest()
            req.action = THROW
            req.limb = self.limb_name
            
            throw_res = self.throw(req)
            return throw_res.success

    
        loginfo("PHASE: 3")
        self.MODE = BLOCK
        self.goal_joints = self.arm.joint_angles()

        self.playing = True
        
        while not rospy.is_shutdown() and self.playing:

            if self.MODE == BLOCK :
                loginfo("MODE: BLOCK")
                # returns true when its sure the ball not going in the goal and still on our side,
                # false if on other side
                blocked = _BLOCK()
                if blocked:
                    self.MODE = GRAB

                
            elif self.MODE == GRAB :
                loginfo("MODE: GRAB")
                # returns true if grabbing was successful, false if the ball goes on the other side
                grabbed = _GRAB()
                if grabbed:
                    self.MODE = THROW
                else:
                    self.MODE = BLOCK

                    
            # elif self.MODE == CHECK_BLOCKS :
            #     loginfo("MODE: CHECK_BLOCKS")
            #     # moves the arm out of the way and then uses vision to find the blocks
            #     # returns true if the blocks are in the right place, false if they need to be moved
            #     blocks_ok = CHECK_BLOCKS()
            #     if blocks_ok:
            #         self.MODE = THROW
            #     else:
            #         self.MODE = MOVE_BLOCKS

                
            # elif self.MODE == MOVE_BLOCKS :
            #     loginfo("MODE: MOVE_BLOCKS")
            #     # returns after the the blocks have been moved
            #     MOVE_BLOCKS()
            #     self.MODE = GRAB

                
            elif self.MODE == THROW :
                loginfo("MODE: THROW")
                # returns after the throw
                _THROW()
                self.MODE = BLOCK

            self.rate.sleep()

    def GameLoop(self):
        while not rospy.is_shutdown():
            if self.PHASE == 1 :
                # returns after calibration
                self.PHASE1()
                self.PHASE = 2

            elif self.PHASE == 2 :
                # returns after the the blocks have been moved
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
    ct = controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

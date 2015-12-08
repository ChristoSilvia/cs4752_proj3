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
import tf2_ros
from tf.transformations import *
from copy import deepcopy
import cv2



def loginfo(logstring):
    rospy.loginfo("Controller: {0}".format(logstring))

class controller() :
    def __init__(self, limb):
        rospy.init_node('controller')
        loginfo("Initialized node Controller")

        # rospy.Subscriber("/plane_traj", Trajectory, self.plane_trajCb, queue_size=10000)
        # self.move_plane_traj_srv = createService('move_plane_traj', JointAction, self.handle_move_plane_traj, "left")

        # self.set_plane_normal_srv = createServiceProxy('set_normal_vec', SetNormalVec, "")
        
        baxter_interface.RobotEnable(CHECK_VERSION).enable()


        
        self.move_robot = createServiceProxy("move_robot", MoveRobot, "")
        # self.move_robot_plane_service = createService('move_robot_plane', MoveRobot, self.handle_move_robot_plane, "")


        # self.joint_action_server = createServiceProxy("move_end_effector_trajectory", JointAction, "left")
        # self.position_server = createServiceProxy("end_effector_position", EndEffectorPosition, "left")

        # self.get_block_poses = createServiceProxy("get_block_poses", BlockPoses, "")

        # self.tf_br = tf2_ros.TransformBroadcaster()

        #Generate the figure
        # self.fig = plt.figure()
        # self.ax = self.fig.add_subplot(111)
        # self.ax.hold(True)
        # plt.show()

        self.block_size = .045
        self.limb = rospy.get_param("limb")
        self.arm = baxter_interface.Limb(self.limb)
        self.num_blocks = rospy.get_param("num_blocks")
        self.PHASE = 1

        self.playing_field = {}
        self.rate = rospy.Rate(1)
        loginfo("Starting GameLoop")
        self.GameLoop()

        rospy.spin()

    # def sendTransform(self):
    #     self.transform_setup()        

    #     t = TransformStamped()
    #     t.header.stamp = rospy.Time.now()
    #     t.header.frame_id = "base"
    #     t.child_frame_id = "plane_frame"
    #     T = translation_from_matrix(self.plane_translation)
    #     t.transform.translation.x = T[0]
    #     t.transform.translation.y = T[1]
    #     t.transform.translation.z = T[2]
    #     q = quaternion_from_matrix(self.plane_rotation)
    #     t.transform.rotation.x = q[0]
    #     t.transform.rotation.y = q[1]
    #     t.transform.rotation.z = q[2]
    #     t.transform.rotation.w = q[3]

    #     self.tf_br.sendTransform(t)

    def PHASE1(self):
        loginfo("PHASE: 1")

        def get_base_frame_points(self):
            calibration_points = ["BOTTOM_MIDDLE", "BOTTOM_CORNER", "BALL_START", "TOP_CORNER"]
            point_pos = {}
            print calibration_points
            for point_name in calibration_points:
                prompt = "Press Enter when Arm is on the %s" % point_name
                cmd = raw_input(prompt)
                point_pos[point_name] = np.array(self.arm.endpoint_pose()['position'])
            return point_pos

        def get_kinect_frame_points(self):
            calibration_points = ["BOTTOM_MIDDLE", "BOTTOM_CORNER", "BALL_START", "TOP_CORNER", "BOTTOM_B_NEAR_GOAL", "TOP_B_NEAR_GOAL"]

            # img = cv2.imread('dave.jpg')
            # gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            # edges = cv2.Canny(gray,50,150,apertureSize = 3)
            # minLineLength = 100
            # maxLineGap = 10
            # lines = cv2.HoughLinesP(edges,1,np.pi/180,100,minLineLength,maxLineGap)
            # for x1,y1,x2,y2 in lines[0]:
            # cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)


            
            # TODO use vision to find all points
            # indexes = {
            #     "BOTTOM_CORNER": 0,
            #     "BOTTOM_B_NEAR_GOAL": 1,
            #     "BOTTOM_B_NEAR_MIDDLE": 2,
            #     "BOTTOM_MIDDLE": 3,
            #     "TOP_CORNER": 4,
            #     "TOP_B_NEAR_GOAL": 5,
            #     "TOP_B_NEAR_MIDDLE": 6,
            #     "TOP_MIDDLE": 7,
            #     "GOAL": 8,
            #     "BALL_START": 9,
            #     "BLOCK_START": 10
            # }
            hardcoded_points = [
                np.array([0.5,0.0,1.0]),
                np.array([0.0,0.0,1.0]),
                np.array([0.0,0.0,1.0]),
                np.array([0.0,0.0,1.0]),
                np.array([0.0,0.0,1.0]),
                np.array([0.4,-0.2,1.0])
            ]
            point_pos = {}
            for i in range(0,len(calibration_points)):
                point_name = calibration_points[i]
                point_pos[point_name] = hardcoded_points[i]
            return point_pos


        def get_kinect_transform(base_points, kinect_points):
            shape = (1, len(base_points), 3)
            source = np.zeros(shape, np.float32)
            target = np.zeros(shape, np.float32)

            count = 0
            for point_name in base_points:
                kinect_pt = list(kinect_points[point_name])
                source[0][count] = kinect_pt
                base_pt = list(base_points[point_name])
                target[0][count] = base_pt
                count += 1
            # print source
            # print target

            retval, M, inliers = cv2.estimateAffine3D(source, target)
            M = np.append(M,[[0,0,0,1]], axis=0)

            return M

        def KinectToBasePoint(M, kinect_x,kinect_y,kinect_z):
            # self.transform_setup()
            # M = np.dot(self.kinect_translation, self.kinect_rotation)

            kinect_coords = np.array([kinect_x,kinect_y,kinect_z,1])
            base_coords = np.dot(M, kinect_coords.T)
            base_coords = base_coords[:3]/base_coords[3]
            base_coords.reshape((1, 3))
            # print "base_coords: {0}".format(base_coords)
            return base_coords

        def calibrate(self):
            loginfo("Calibrating Playing Field and Kinect Frame")
            
            base_points = get_base_frame_points(self)
            # print base_points

            prompt = "Press Enter when Arm is out of the way of the kinect's view of the table"
            cmd = raw_input(prompt)

            kinect_points = get_kinect_frame_points(self)
            # print kinect_points

            kinect_transform = get_kinect_transform(base_points, kinect_points)
            print kinect_transform
            
            self.playing_field = {}
            for point_name in kinect_points:
                kinect_pt = kinect_points[point_name]
                base_pt = KinectToBasePoint(kinect_transform, kinect_pt[0],kinect_pt[1],kinect_pt[2])
                self.playing_field[point_name] = base_pt
            print self.playing_field

            # vec1 = point_pos[1] - point_pos[0]
            # vec2 = point_pos[2] - point_pos[0]
            
            # self.kinect_norm = np.cross(vec1, vec2)
            # self.set_plane_normal_srv(Vector3(self.plane_norm[0], self.plane_norm[1], self.plane_norm[2]))
            # # plane_origin = np.average(point_pos, axis=0)
            # plane_origin = point_pos[0]

            # self.plane_translation = translation_matrix(plane_origin)
            
            # x_plane = vec1/np.linalg.norm(vec1)
            # y_plane = np.cross(self.plane_norm, vec1)
            # y_plane = y_plane/np.linalg.norm(y_plane)
            # z_plane = self.plane_norm/np.linalg.norm(self.plane_norm)
            # #need rotation to make norm the z vector
            # self.plane_rotation = np.array([x_plane, y_plane, z_plane]).T

            # self.plane_rotation = np.append(self.plane_rotation,[[0,0,0]],axis=0)
            # self.plane_rotation = np.append(self.plane_rotation,[[0],[0],[0],[1]],axis=1)
            
            print "#################################"
            print "Finished Calibrating Playing Field and Kinect Frame"
            print "kinect_translation"
            print translation_from_matrix(kinect_transform)
            print "kinect_rotation"
            print euler_from_matrix(kinect_transform)
            print "#################################"

            self.calibrated_plane = True

            # self.sendTransform()

        self.calibrated_plane = False
        self.plane_norm = Vector3()
        calibrate(self)



    
    def PHASE2(self):
        loginfo("PHASE: 2")


        def initBlockPositions(self):
            rospy.loginfo("Initializing block positions")
            self.initial_pose = Pose()
            pos = np.array(self.arm.endpoint_pose()['position'])
            ori = np.array(self.arm.endpoint_pose()['orientation'])
            self.initial_pose.position = Point(pos[0],pos[1],pos[2])
            self.initial_pose.orientation = Quaternion(ori[0],ori[1],ori[2],ori[3])
            # print self.initial_pose

            block_poses = []
            for i in range(0,self.num_blocks) :
                bp = deepcopy(self.initial_pose)
                bp.position.z -=  i * self.block_size
                block_poses.append(bp)
            # print self.block_poses
            return block_poses


        block_poses = initBlockPositions(self)
        # resp = self.move_robot(req.action, req.limb, base_pose)
        green_B = deepcopy(self.initial_pose)
        B_center = (self.playing_field["BOTTOM_B_NEAR_GOAL"] + self.playing_field["TOP_B_NEAR_GOAL"]) / 2.
        green_B.position = Point(B_center[0],B_center[1],B_center[2])
        print green_B

        self.desired_block_poses = []
        inc = self.block_size + self.block_size/2
        length = (self.num_blocks - 1) * inc
        for i in range(0,self.num_blocks) :
            bp = deepcopy(green_B)
            bp.position.x +=  i * inc - length/2
            self.desired_block_poses.append(bp)


        for i in range(0,self.num_blocks):
            if i != 0:
                self.move_robot(MOVE_TO_POSE_INTERMEDIATE, self.limb, block_poses[i])

            self.move_robot(CLOSE_GRIPPER, self.limb, Pose())

            self.move_robot(MOVE_TO_POSE_INTERMEDIATE, self.limb, self.desired_block_poses[i])

            self.move_robot(OPEN_GRIPPER, self.limb, Pose())


    def PHASE3(self):

        def BLOCK(self):
            pass

        def GRAB(self):
            return True
            # return False

        def CHECK_BLOCKS(self):
            # block_poses = self.get_block_poses(self.num_blocks)
            return True
            # return False

        def MOVE_BLOCKS(self):
            pass

        def THROW(self):
            pass

    
        loginfo("PHASE: 3")
        self.MODE = BLOCK

        playing = True
        
        while not rospy.is_shutdown() and playing:

            if self.MODE == BLOCK :
                loginfo("MODE: BLOCK")
                # returns true when its sure the ball not going in the goal and still on our side
                BLOCK(self)
                self.MODE = GRAB

                
            elif self.MODE == GRAB :
                loginfo("MODE: GRAB")
                # returns true if grabbing was successful, false if the ball goes on the other side
                grabbed = GRAB(self)
                if grabbed:
                    self.MODE = CHECK_BLOCKS
                else:
                    self.MODE = BLOCK

                    
            elif self.MODE == CHECK_BLOCKS :
                loginfo("MODE: CHECK_BLOCKS")
                # moves the arm out of the way and then uses single color vision to find the blocks
                # returns true if the blocks are in the right place, false if they need to be moved
                blocks_ok = CHECK_BLOCKS(self)
                if blocks_ok:
                    self.MODE = THROW
                else:
                    self.MODE = MOVE_BLOCKS

                
            elif self.MODE == MOVE_BLOCKS :
                loginfo("MODE: MOVE_BLOCKS")
                # returns after the the blocks have been moved
                MOVE_BLOCKS(self)
                self.MODE = GRAB

                
            elif self.MODE == THROW :
                loginfo("MODE: THROW")
                # returns after the throw
                THROW(self)
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

            self.rate.sleep()

    def get_tool_pos(self):
        tool_vec = self.position_server().position
        # tool_vec = self.tool_position_server().position
        tool = np.array([tool_vec.x,tool_vec.y,tool_vec.z])
        return tool


    def matrix_from_euler(self, r,p,y,radians=False):
        if not radians:
            a = math.pi/180.
            r *= a
            p *= a
            y *= a

        q = quaternion_from_euler(r, p, y)
        return quaternion_matrix(q)

    def transform_setup(self):
        try:
            self.plane_translation
            self.plane_rotation
        except AttributeError:
            loginfo("Getting Current Pos Because Plane Not Calibrated")
            p = self.position_server().position
        
            self.plane_translation = translation_matrix([p.x,p.y,p.z])
            # self.plane_translation = translation_matrix(self.get_tool_pos())
            # self.plane_rotation = np.identity(4)
            self.plane_rotation = self.matrix_from_euler(0,0,-90)

    def PlaneToBasePoint(self, plane_x,plane_y,plane_z):
        self.transform_setup()

        M = np.dot(self.plane_translation, self.plane_rotation)

        plane_coords = np.array([plane_x,plane_y,plane_z,1])
        base_coords = np.dot(M, plane_coords.T)
        base_coords = base_coords[:3]/base_coords[3]
        base_coords.reshape((1, 3))
        # print "base_coords: {0}".format(base_coords)
        return base_coords

    def PlaneToBaseDir(self, plane_x,plane_y,plane_z):
        self.transform_setup()

        plane_dir = np.array([plane_x,plane_y,plane_z,1])
        base_dir = np.dot(self.plane_rotation, plane_dir.T)
        base_dir = base_dir[:3]/base_dir[3]
        base_dir.reshape((1, 3))
        # print "base_dir: {0}".format(base_dir)
        return base_dir

    def PlaneToBaseOrientation(self, plane_x,plane_y,plane_z,plane_w):
        self.transform_setup()
        plane_orientation = quaternion_matrix([plane_x,plane_y,plane_z,plane_w])
        po = quaternion_matrix([plane_x,plane_y,plane_z,plane_w])
        bo = np.dot(po, self.plane_rotation)
        return quaternion_from_matrix(bo)

    def handle_move_robot_plane(self, req):
        pp = req.pose.position
        po = req.pose.orientation
        wp = self.PlaneToBasePoint(pp.x,pp.y,pp.z+zoffset/2)
        wo = self.PlaneToBaseOrientation(po.x,po.y,po.z,po.w)
        base_pose = Pose()
        base_pose.position = Vector3(wp[0],wp[1],wp[2])
        base_pose.position = Vector3(wp[0],wp[1],wp[2])
        base_pose.orientation = Quaternion(wo[0],wo[1],wo[2],wo[3])
        resp = self.move_robot(req.action, req.limb, base_pose)
        return MoveRobotResponse(resp.success)

    def vector3_to_np(self,v):
        return np.array([v.x,v.y,v.z])

    def handle_move_plane_traj(self, req):
        msg = Trajectory()
        msg.times = req.times
        msg.positions = req.positions
        msg.velocities = req.velocities
        self.plane_trajCb(msg)
        return JointActionResponse()

    def plane_trajCb(self, plane_traj_msg):
        self.got_plane_traj = True
        T = np.array(plane_traj_msg.times)
        P = np.zeros([0,3])
        V = np.zeros([0,3])

        positions = []
        velocities = []
        for i in range(0,len(plane_traj_msg.positions)):
            pp = plane_traj_msg.positions[i]
            wp = self.PlaneToBasePoint(pp.x,pp.y,pp.z+zoffset)
            pp = self.vector3_to_np(pp).reshape((1, 3))
            P = np.append(P, pp, axis=0)
            positions.append(Vector3(wp[0],wp[1],wp[2]))

            pv = plane_traj_msg.velocities[i]
            wv = self.PlaneToBaseDir(pv.x,pv.y,pv.z)
            pv = self.vector3_to_np(pv).reshape((1, 3))
            V = np.append(V, pv, axis=0)
            velocities.append(Vector3(wv[0],wv[1],wv[2]))

        print "############### Recieved plane_traj_msg ##################"
        print "len(times): %d" % len(plane_traj_msg.times)
        print "len(positions): %d" % len(positions)
        print "len(velocities): %d" % len(velocities)
        print "shapes:"
        print T.shape
        print P.shape
        print V.shape
        print "################ T #################"
        print T
        print "################ P #################"
        print P
        print "################ V #################"
        print V
        print "##########################################################"

        self.ax.plot(P[:,0],P[:,1])
        self.fig.canvas.draw()

        force_control = False
        if force_control:
            self.draw_on_plane(plane_traj_msg.times, positions, velocities)
        else:
            self.joint_action_server(plane_traj_msg.times, positions, velocities)



if __name__ == '__main__':
    limb = 'left'
    ct = controller(limb)

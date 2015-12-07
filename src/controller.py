#!/usr/bin/env python
# Team: zic; Names: Zach Vinegar, Isaac Qureshi, Chris Silvia
import rospy
import numpy as np
from scipy.spatial import KDTree
from scipy.interpolate import PiecewisePolynomial
from matplotlib import pyplot as plt
from cs4752_proj2.msg import *
from cs4752_proj2.srv import *
from config import *
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker, MarkerArray
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
import tf
import tf2_ros
from tf.transformations import *
from copy import deepcopy

zoffset = -.007
# zoffset = 0

def loginfo(logstring):
    rospy.loginfo("Controller: {0}".format(logstring))

class controller() :
    def __init__(self):
        rospy.init_node('controller')
        loginfo("Initialized node Controller")

        rospy.Subscriber("/plane_traj", Trajectory, self.plane_trajCb, queue_size=10000)
        self.move_plane_traj_srv = createService('move_plane_traj', JointAction, self.handle_move_plane_traj, "left")

        self.set_plane_normal_srv = createServiceProxy('set_normal_vec', SetNormalVec, "")
        
        baxter_interface.RobotEnable(CHECK_VERSION).enable()
        
        self.move_robot = createServiceProxy("move_robot", MoveRobot, "")
        self.move_robot_plane_service = createService('move_robot_plane', MoveRobot, self.handle_move_robot_plane, "")


        self.joint_action_server = createServiceProxy("move_end_effector_trajectory", JointAction, "left")
        self.draw_on_plane = createServiceProxy("draw_on_plane", JointAction, "left")
        self.position_server = createServiceProxy("end_effector_position", EndEffectorPosition, "left")
        # self.tool_trajectory = createServiceProxy("move_tool_trajectory", JointAction, "left")
        # self.tool_position_server = createServiceProxy("tool_position", EndEffectorPosition, "left")

        # self.tf_br = tf2_ros.TransformBroadcaster()

        self.got_plane_traj = False
        self.calibrated_plane = False
        self.plane_norm = Vector3()
        self.calibrate_plane()

        #Generate the figure
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.ax.hold(True)
        plt.show()



        # rate = rospy.Rate(30)
        # while not rospy.is_shutdown():
        #     if self.got_plane_traj or self.calibrated_plane:
        #         self.sendTransform()
        #     rate.sleep()

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

    def calibrate_plane(self):
        point_count = 0
        point_pos = []
        print "Calibrating Plane"
        print "Go left to right from pt1 to pt2 and then up to pt3"
        while point_count < 3 :
            prompt = "Press Enter when Arm is on the %d plane point" % point_count
            cmd = raw_input(prompt)
            point_pos.append(self.get_tool_pos())
            print point_pos[point_count]
            point_count += 1

        vec1 = point_pos[1] - point_pos[0]
        vec2 = point_pos[2] - point_pos[0]
        
        self.plane_norm = np.cross(vec1, vec2)
        self.set_plane_normal_srv(Vector3(self.plane_norm[0], self.plane_norm[1], self.plane_norm[2]))
        # plane_origin = np.average(point_pos, axis=0)
        plane_origin = point_pos[0]

        self.plane_translation = translation_matrix(plane_origin)
        
        x_plane = vec1/np.linalg.norm(vec1)
        y_plane = np.cross(self.plane_norm, vec1)
        y_plane = y_plane/np.linalg.norm(y_plane)
        z_plane = self.plane_norm/np.linalg.norm(self.plane_norm)
        #need rotation to make norm the z vector
        self.plane_rotation = np.array([x_plane, y_plane, z_plane]).T

        self.plane_rotation = np.append(self.plane_rotation,[[0,0,0]],axis=0)
        self.plane_rotation = np.append(self.plane_rotation,[[0],[0],[0],[1]],axis=1)
        
        print "#################################"
        print "Finished Calibrating Plane"
        print "self.plane_translation"
        print translation_from_matrix(self.plane_translation)
        print "self.plane_rotation"
        print self.plane_rotation
        # print euler_from_matrix(self.plane_rotation)
        print "#################################"

        self.calibrated_plane = True

        # self.sendTransform()


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
    ct = controller()

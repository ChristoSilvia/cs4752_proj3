#!/usr/bin/env python
# Team: zic; Names: Zach Vinegar, Isaac Qureshi, Chris Silvia
import rospy
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from cs4752_proj2.srv import *
from cs4752_proj2.msg import *
from baxter_core_msgs.msg import *
from baxter_core_msgs.srv import *
from baxter_interface import *
from config import *
from copy import deepcopy
from tf.transformations import *

class RobotInterface():
    def __init__(self):
        rospy.init_node('position_control')
        rospy.loginfo("Initialized Position Control")

        rospy.loginfo("Beginning to enable robot")
        self.baxter = RobotEnable()
        rospy.loginfo("Enabled Robot")
        
        self.hand_pose_left = Pose()
        self.hand_pose_right = Pose()
        
        self.gripper_left = Gripper('left')
        self.gripper_right = Gripper('right')

        rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, self.respondToEndpointLeft)
        rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, self.respondToEndpointRight)
        
        move_robot_service = createService('move_robot', MoveRobot, self.handle_move_robot, "")

        try :
            rospy.loginfo("Initializing service proxy for /SolvePositionIK...")
            ns = "ExternalTools/left/PositionKinematicsNode/IKService"
            rospy.wait_for_service(ns, 5.0)
            self.iksvc_left = rospy.ServiceProxy(ns, SolvePositionIK)
            ns = "ExternalTools/right/PositionKinematicsNode/IKService"
            rospy.wait_for_service(ns, 5.0)
            self.iksvc_right = rospy.ServiceProxy(ns, SolvePositionIK)
            rospy.loginfo("Initialized service proxy for /SolvePositionIK...")
        except rospy.ServiceException, e:
            rospy.logerr("Service Initializing Failed: {0}".format(e))

        print "Ready to move robot."

        rospy.spin()

    def respondToEndpointLeft(self, EndpointState) :
        self.hand_pose_left = deepcopy(EndpointState.pose)

    def respondToEndpointRight(self, EndpointState) :
        self.hand_pose_right = deepcopy(EndpointState.pose)

    def handle_move_robot(self, req):
        rospy.sleep(1)
        success = True
        gripper = self.gripper_left if req.limb == 'left' else self.gripper_right
        
        if not (req.limb == 'left' or req.limb == 'right'):
            rospy.logerr("No Limb Set")

        if req.action == OPEN_GRIPPER:
            rospy.loginfo("Beginning to open gripper")
            # rospy.sleep(GRIPPER_WAIT)
            gripper.open(block=True)
            # rospy.sleep(GRIPPER_WAIT)
            rospy.loginfo("Opened Gripper")

        elif req.action == CLOSE_GRIPPER :
            rospy.loginfo("Beginning to close Gripper")
            # rospy.sleep(GRIPPER_WAIT)
            gripper.close(block=True)
            # rospy.sleep(GRIPPER_WAIT)
            rospy.loginfo("Closed Gripper")

        elif req.action == MOVE_TO_POSE_INTERMEDIATE :
            rospy.loginfo("Trying to Move To Pose")
            success = self.MoveToPoseWithIntermediate(req.limb, req.pose)

        elif req.action == MOVE_TO_POSE :
            rospy.loginfo("Trying to Move To Pose")
            success = self.MoveToPose(req.limb, req.pose, "FAILED MoveToPose")

        elif req.action == MOVE_TO_POS :
            rospy.loginfo("Trying to Move To Pos")

            new_pose = Pose()
            if req.limb == 'left':
                try:
                    self.initial_left
                    new_pose = deepcopy(self.initial_left)
                except AttributeError:
                    new_pose = deepcopy(self.hand_pose_left)
                    self.initial_left = deepcopy(self.hand_pose_left)
            elif req.limb == 'right':
                try:
                    self.initial_right
                    new_pose = deepcopy(self.initial_right)
                except AttributeError:
                    new_pose = deepcopy(self.hand_pose_right)
                    self.initial_right = deepcopy(self.hand_pose_right)

            new_pose.position = deepcopy(req.pose.position)
            # success = self.MoveToPose(req.limb, new_pose, "FAILED MoveToPose")
            success = self.MoveToPoseWithIntermediate(req.limb, new_pose)
            rospy.loginfo("Moved to pos: %r" % success)

        else :
            print "invalid action"

        return MoveRobotResponse(success)

    def MoveToPoseWithIntermediate(self, limb, pose, inter1=True, inter2=True, inter3=False) :
        hand_pose = self.hand_pose_left if limb == 'left' else self.hand_pose_right
        if inter1 :
            interpose1 = self.getOffsetPose(hand_pose, .05)
            b1 = self.MoveToPose(limb, interpose1, "MoveToIntermediatePose")
        if inter2 :
            interpose2 = self.getOffsetPose(pose, .05)
            b2 = self.MoveToPose(limb, interpose2, "MoveToIntermediatePose")
        if inter3 :
            interpose2 = self.getOffsetPose(pose, .01)
            b3 = self.MoveToPose(limb, interpose2, "MoveToRightAbovePose")
        
        b4 = self.MoveToPose(limb, pose, "MoveToPose")
        return b4

    def MoveToPose(self, limb, pose, name) :
        joint_solution = self.inverse_kinematics(limb, pose)
        if joint_solution != [] :
            self.moveArm(limb, joint_solution)
            rospy.loginfo("SUCCEEDED: %s" % name)
            # rospy.sleep(MOVE_WAIT)
            return True
        else :
            rospy.logerr("FAILED %s" % name)
            return False

    def getOffsetPose(self, pose, offset) :
        offsetpose = deepcopy(pose)
        q = pose.orientation
        off = np.dot(
            quaternion_matrix([q.x,q.y,q.z,q.w]),
            np.array([0,0,-offset,1]).T)
        off = off[:3]/off[3]
        offsetpose.position.x += off[0]
        offsetpose.position.y += off[1]
        offsetpose.position.z += off[2]
        return offsetpose

    def moveArm (self, limb, joint_solution) :
        arm = Limb(limb)
        arm.move_to_joint_positions(joint_solution)
        rospy.sleep(0.01)

    def inverse_kinematics(self, limb, ourpose) :
        ikreq = SolvePositionIKRequest()

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        poses = {
            limb : PoseStamped(
                header = hdr,
                pose = ourpose
            ),
        }         
        ikreq.pose_stamp.append(poses[limb])

        iksvc = self.iksvc_left if limb == 'left' else self.iksvc_right

        try :
            ns = "ExternalTools/"+limb+"/PositionKinematicsNode/IKService"
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return []
        if (resp.isValid[0]):
            print("SUCCESS - Valid Joint Solution Found:")
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            return limb_joints
        else :
            rospy.logerr("Invalid pose")
            return []

if __name__ == '__main__':
    try:
        RobotInterface()
    except rospy.ROSInterruptException:
        pass

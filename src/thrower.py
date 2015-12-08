#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from cs4752_proj3.msg import *
from cs4752_proj3.srv import *
from baxter_core_msgs.msg import *
from baxter_core_msgs.srv import *
import baxter_interface
from baxter_interface import *
import baxter_external_devices
from config import *
from copy import deepcopy

class thrower :
	def __init__(self):
		rospy.init_node('thrower')
		rospy.loginfo("Initialized Thrower")

		rs = baxter_interface.RobotEnable(CHECK_VERSION)
		init_state = rs.state().enabled

		self.move_robot = createServiceProxy("move_robot", MoveRobot, "")

		self.limb = "left"
		self.gripper = baxter_interface.Gripper(self.limb, CHECK_VERSION)

		self.arm = Limb(self.limb)

		release_pos = Point(0.573,0.081,0.036)
		release_vel = Vector3(1,1,0)
		
		# move to the release pose
		release_pose = Pose()
		release_pose.position = release_pos
		release_pose.orientation = Quaternion(0.140770659119,0.989645234506,0.0116543447684,0.0254972076605)
		self.MoveToPose(release_pose)
		
		# close the gripper
		self.move_robot(CLOSE_GRIPPER, self.limb, Pose())

		self.throw(release_pos, release_vel)

		#rospy.spin()

	def MoveToPose(self, pose) :
		rospy.loginfo("Going to Pose")
		# success = MoveToPose(homepose, False, False, False)
		success = self.move_robot(MOVE_TO_POSE, self.limb, pose)
		rospy.loginfo("Got to Pose : %r", success)

	def respondToEndpoint(self, EndpointState) :
		self.hand_pose = deepcopy(EndpointState.pose)

	def throw(self, release_pos, release_vel) :

		speed = -10.0
		joint_names = ["s0", "s1", "e0", "e1","w0", "w1", "w2"]
		throw = [0,0,0,0,0, speed,0]
		zero = [0,0,0,0,0,0,0]
		throw_dict = {}
		zero_dict = {}

		for i in xrange(0,7) :
			throw_dict[self.limb+'_'+joint_names[i]] = throw[i]
			zero_dict[self.limb+'_'+joint_names[i]] = 0


		current_joints = self.arm.joint_angles()
		back_swing = .7
		follow_through = 1.2
		release_offset = .6
		release_angle = current_joints[self.limb+'_w1'] + release_offset
		current_joints[self.limb+'_w1'] = current_joints[self.limb+'_w1'] + back_swing
		current_joints[self.limb+'_w2'] = 0.0
		self.arm.move_to_joint_positions(current_joints)

		rospy.sleep(.5)

		current_angle = current_joints[self.limb+'_w1']


		currenttime = rospy.get_time()
		# endswing = currenttime + swing_time
		#releasetime = currenttime + release_time
		released = False
		rate = rospy.Rate(60)
		# oldz = 10000000
		while current_angle > release_angle-follow_through :
			#releases when hand starts going up again
			# print current_angle
			#print self.hand_pose.position.z
			if release_angle > current_angle and not released : 
				# print "Beginning gripper open"
				self.gripper.command_position(100, block=False)
				released = True
				#rospy.sleep(.05)
				# print "gripper should be open"
			self.arm.set_joint_velocities(throw_dict)
			#oldz = self.hand_pose.position.z
			current_joints = self.arm.joint_angles()
			current_angle = current_joints[self.limb+'_w1']
			#print oldz
			rate.sleep()

		#rospy.sleep(.01)
		#self.gripper.command_position( 100, block=False)
		#rospy.sleep(2)
		
		self.arm.set_joint_velocities(zero_dict)


if __name__ == '__main__':
	try:
		thrower()
	except rospy.ROSInterruptException:
		pass

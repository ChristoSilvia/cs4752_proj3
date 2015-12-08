#!/usr/bin/env python
import rospy
import math
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

# source: http://sdk.rethinkrobotics.com/wiki/Hardware_Specifications

# Maximum Joint Speeds (rad/sec)
# Joint	Maximum Speed
# S0	2.0
# S1	2.0
# E0	2.0
# E1	2.0
# W0	4.0
# W1	4.0
# W2	4.0

# Joint Range Table (Bend Joints)
# Joint	Min limit	Max limit	Range (Radians)
# S1	-2.147		+1.047		3.194
# E1	-0.052		+2.618		2.67
# W1	-1.571		+2.094		3.665


# Joint Range Table (Twist Joints)
# Joint	Min limit	Max limit	Range (Radians) 
# E0	-3.028		+3.028		6.056
# S0	-2.461		+0.890		3.351
# W0	-3.059		+3.059		6.117
# W2	-3.059		+3.059		6.117

class thrower :
	def __init__(self):
		rospy.init_node('thrower')
		rospy.loginfo("Initialized Thrower")

		rs = baxter_interface.RobotEnable(CHECK_VERSION)
		init_state = rs.state().enabled

		self.move_robot = createServiceProxy("move_robot", MoveRobot, "")

        self.limb = rospy.get_param("limb")
		self.gripper = baxter_interface.Gripper(self.limb, CHECK_VERSION)

		self.arm = Limb(self.limb)

		# test throw params
		release_pos = Point(0.573, 0.081, 0.036)
		release_vel = Vector3(0.30, 0.40, 0.00)
		
		# set the release pose
		release_pose = Pose()
		release_pose.position = release_pos
		release_pose.orientation = Quaternion(0.140770659119,0.989645234506,0.0116543447684,0.0254972076605)
		
		# move to the release pose
		success = self.move_robot(MOVE_TO_POSE, self.limb, release_pose)
		
		# close the gripper
		self.move_robot(CLOSE_GRIPPER, self.limb, Pose())

		# throw the ball
		self.throw(release_pos, release_vel)

		#rospy.spin()

	def throw(self, release_pos, release_vel) :
		safty_buffer = math.radians(10.0)
		w1_min = -1.571
		w1_max = 2.094

		linear_speed = np.linalg.norm(release_vel)
		print "linear_speed (m/sec)"
		print linear_speed

		link_len = 0.29 # need to measure
		angular_speed = linear_speed/link_len
		print "angular_speed (deg/sec)"
		print  math.degrees(angular_speed)

		throw = [0,0,0,0,0,angular_speed,0]
		zero = [0,0,0,0,0,0,0]
		throw_dict = {}
		zero_dict = {}

		for i in xrange(0,7) :
			throw_dict[self.limb+'_'+joint_names[i]] = throw[i]
			zero_dict[self.limb+'_'+joint_names[i]] = 0

		back_swing = math.radians(40.0)
		follow_through = math.radians(60.0)
		open_gripper_time = .10 # need to calibrate

		current_joints = self.arm.joint_angles()

		# set release_angle as current joint angle
		release_angle = current_joints[self.limb+'_w1']

		# set open_angle to compensate for open gripper time
		open_offset = open_gripper_time * angular_speed
		open_angle = release_angle + open_offset

		# set stop_angle allowing for follow through 
		stop_angle = release_angle - follow_through
		# make sure follow through won't hit joint limit
		if stop_angle < w1_min + safty_buffer :
			stop_angle = w1_min + safty_buffer

		# move the wrist bend to backswing pos
		# move the wrist twist to neutral pos
		current_joints[self.limb+'_w1'] += back_swing
		current_joints[self.limb+'_w2'] = 0.0
		self.arm.move_to_joint_positions(current_joints)

		rospy.sleep(.5)

		opened = False
		released = False
		rate = rospy.Rate(60)
		while True :
			current_joints = self.arm.joint_angles()
			current_angle = current_joints[self.limb+'_w1']

			# opens gripper slightly before release_angle to account for open gripper time
			if not opened and open_angle > current_angle : 
				self.gripper.command_position(100, block=False)
				opened = True
				print "open_angle"
				print open_angle
				print "actual_open_angle"
				print current_angle

			# should release at correct pos/vel
			if not released and release_angle > current_angle :
				actual_release_pos = np.array(self.arm.endpoint_pose()['position'])
				actual_release_vel = np.array(self.arm.endpoint_velocity()['linear'])
				actual_release_speed = np.linalg.norm(actual_release_vel)
				released = True
				print "actual_release_pos"
				print actual_release_pos
				print "actual_release_speed"
				print actual_release_speed

			# stops at the stop_angle
			if stop_angle > current_angle :
				self.arm.set_joint_velocities(zero_dict)
				break

			self.arm.set_joint_velocities(throw_dict)
			rate.sleep()



if __name__ == '__main__':
	try:
		thrower()
	except rospy.ROSInterruptException:
		pass

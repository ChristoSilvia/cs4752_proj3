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
from tf.transformations import *
import baxter_pykdl

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

# Joint Range Table
# Joint	Min limit	Max limit	Range (Radians)
# S0	-2.461		+0.890		3.351
# S1	-2.147		+1.047		3.194
# E0	-3.028		+3.028		6.056
# E1	-0.052		+2.618		2.67
# W0	-3.059		+3.059		6.117
# W1	-1.571		+2.094		3.665
# W2	-3.059		+3.059		6.117

class thrower :
	def __init__(self):
		rospy.init_node('thrower')
		rospy.loginfo("Initialized Thrower")

		rs = baxter_interface.RobotEnable(CHECK_VERSION)
		init_state = rs.state().enabled

		self.move_robot = createServiceProxy("move_robot", MoveRobot, "")

		# self.limb = 'left'
		self.limb = rospy.get_param("limb")
		self.gripper = baxter_interface.Gripper(self.limb, CHECK_VERSION)

		self.arm = Limb(self.limb)

		self.arm_kin = baxter_pykdl.baxter_kinematics(self.limb)

		self.target_pose_pub = rospy.Publisher('zic_target_pose', PoseStamped, queue_size=1)
        
		# set the release pose
		# release_pose = Pose()
		# release_pose.position = release_pos
		# release_pose.orientation = Quaternion(0.140770659119,0.989645234506,0.0116543447684,0.0254972076605)
		# success = self.move_robot(MOVE_TO_POSE, self.limb, release_pose)
		
		# move to the release pose
		# self.moveToThrow()

		# recorded good throw configs:
		self.moveToThrow(w1=-0.7)
		# self.moveToThrow(w1=-0.7, e1=2.40)
		# self.moveToThrow(w1=-0.7, e1=1.57)

		# self.arm.move_to_neutral()

		# print self.arm.joint_angles()
		
		# close the gripper
		# self.move_robot(CLOSE_GRIPPER, self.limb, Pose())

		# throw the ball
		self.throw()
		# self.throw_2([0.5, 0.0, 0.1], 1.0, 0.0, math.radians(10))

		#rospy.spin()

	def moveToThrow(self, e1=2.222, w0=1.7357, w1=0.0000) :
		e1_range = [1.57, 2.22, 2.40] # elbow far or close to body
		w0_range = [0.92, 1.74, 2.50] # yaw for bank shots
		w1_range = [-1.0, 0.00, 0.50] # release pitch

		common_pos = {
			's0': -0.8747, 
			's1': -0.8663,
			'e0': 0.4621, 
			'w2': 0.0000
		}

		new_pos = {}
		for key in common_pos:
			new_pos[self.limb+"_"+key] = common_pos[key]

		new_pos[self.limb+"_e1"] = e1
		new_pos[self.limb+"_w0"] = w0
		new_pos[self.limb+"_w1"] = w1

		if self.limb == 'right':
			new_pos = mirror_left_arm_joints(new_pos)

		self.gripper.command_position(100, block=True)
		self.arm.move_to_joint_positions(new_pos)
		rospy.sleep(1)
		self.gripper.command_position(0, block=True)

	def throw_2(self, release_position, release_speed, release_elevation, release_azimuth):
		link_len = 0.31
		angular_speed = release_speed / link_len

		throw = [0,0,0,0,0,-angular_speed,0]
		zero = [0,0,0,0,0,0,0]
		throw_dict = {}
		zero_dict = {}

		for i in xrange(0,7) :
			throw_dict[self.limb+'_'+joint_names[i]] = throw[i]
			zero_dict[self.limb+'_'+joint_names[i]] = 0

		safty_buffer = math.radians(10.0)
		w1_min = -1.571
		w1_max = 2.094

		back_swing = math.radians(60.0)
		follow_through = math.radians(40.0)
		open_gripper_time = 0.1

		st = np.sin(release_azimuth)
		ct = np.cos(release_azimuth)
		cp = np.cos(release_elevation)
		sp = np.sin(release_elevation)
		rotation_matrix = np.array([[st*cp, -ct, st*sp, 0],
									[-ct*cp, -st, -ct*sp, 0],
									[sp, 0.0, -cp, 0],
									[0,0,0,1]])
			                    
		ron = quaternion_from_matrix(rotation_matrix)
		release_orientation = [ron[1], ron[2], ron[3], ron[0]]

		target_pose = PoseStamped()
		target_pose.header.frame_id = 'base'
		target_pose.header.stamp = rospy.Time.now()
		target_pose.pose = Pose(Vector3(release_position[0], release_position[1], release_position[2]),
							Quaternion(ron[1], ron[2], ron[3], ron[0]))

		self.target_pose_pub.publish(target_pose)
		print(target_pose)
		success = self.move_robot(MOVE_TO_POSE, self.limb, target_pose.pose)


		print(release_position)
		print(release_orientation)

		#throw_beginning_joint_angles_list = self.arm_kin.inverse_kinematics(release_position, orientation=release_orientation)#, seed=[-0.8747, -0.8663, 0.4621, 2.222, 1.7357, 0.0000, 0.0000])

		# set our limb's joint angle to be "back_swing" radians back from the endpoint
		# NOT SURE ABOUT SIGN
		#throw_beginning_joint_angles_list[5] -= back_swing
		print("Beginning to move to joint positions")
		#print(numpy_to_joint_dict(self.limb, throw_beginning_joint_angles_list))
		#self.arm.move_to_joint_positions(numpy_to_joint_dict(self.limb, throw_beginning_joint_angles_list))

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

		opened = False
		released = False
		rate = rospy.Rate(300)
		while True :
			current_joints = self.arm.joint_angles()
			current_angle = current_joints[self.limb+'_w1']

			# opens gripper slightly before release_angle to account for open gripper time
			if not opened and open_angle > current_angle : 
				self.gripper.command_position(100, block=False)
				opened = True
				# print "open_angle"
				# print open_angle
				# print "actual_open_angle"
				# print current_angle

			# should release at correct pos/vel
			if not released and release_angle > current_angle :
				actual_release_pos = np.array(self.arm.endpoint_pose()['position'])
				# actual_release_vel = np.array(self.arm.endpoint_velocity()['linear'])
				actual_release_speed = self.arm.joint_velocities()[self.limb+'_w1'] * link_len
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

        


	def throw(self) :
		# test throw params
		release_pos = Point(0.573, 0.081, 0.036)
		release_vel = Vector3(2.00, 0.00, 0.00)
		
		safty_buffer = math.radians(10.0)
		w1_min = -1.571
		w1_max = 2.094

		linear_speed = np.linalg.norm(vector3_to_numpy(release_vel))
		print "linear_speed (m/sec)"
		print linear_speed

		link_len = 0.31 # need to measure
		angular_speed = linear_speed/link_len
		# print "angular_speed (deg/sec)"
		# print  math.degrees(angular_speed)
		print "angular_speed (rad/sec)"
		print  angular_speed

		throw = [0,0,0,0,0,-angular_speed,0]
		zero = [0,0,0,0,0,0,0]
		throw_dict = {}
		zero_dict = {}

		for i in xrange(0,7) :
			throw_dict[self.limb+'_'+joint_names[i]] = throw[i]
			zero_dict[self.limb+'_'+joint_names[i]] = 0

		back_swing = math.radians(50.0)
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
		if current_joints[self.limb+'_w1'] > w1_max :
			current_joints[self.limb+'_w1'] = w1_max

		current_joints[self.limb+'_w2'] = 0.0
		self.arm.move_to_joint_positions(current_joints)
		rospy.sleep(.5)

		opened = False
		released = False
		rate = rospy.Rate(300)
		while True :
			current_joints = self.arm.joint_angles()
			current_angle = current_joints[self.limb+'_w1']

			# opens gripper slightly before release_angle to account for open gripper time
			if not opened and open_angle > current_angle : 
				self.gripper.command_position(100, block=False)
				opened = True
				# print "open_angle"
				# print open_angle
				# print "actual_open_angle"
				# print current_angle

			# should release at correct pos/vel
			if not released and release_angle > current_angle :
				actual_release_pos = np.array(self.arm.endpoint_pose()['position'])
				# actual_release_vel = np.array(self.arm.endpoint_velocity()['linear'])
				actual_release_speed = self.arm.joint_velocities()[self.limb+'_w1'] * link_len
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
		t = thrower()
	except rospy.ROSInterruptException:
		pass

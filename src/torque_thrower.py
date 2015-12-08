#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from baxter_core_msgs.msg import *
from baxter_core_msgs.srv import *
import baxter_interface
from baxter_interface import *
import baxter_external_devices
from copy import deepcopy
 
from baxter_interface import CHECK_VERSION



class torque_thrower :
	def __init__(self):
		rospy.init_node('torque_thrower')
		rospy.loginfo("Initialized Torque thrower")

		#baxter_interface.RobotEnable(CHECK_VERSION).enable()
		#self.baxter = RobotEnable()
		limb = "left"
		rs = baxter_interface.RobotEnable(CHECK_VERSION)
		init_state = rs.state().enabled
		self.gripper = baxter_interface.Gripper(limb, CHECK_VERSION)
		speed = 4

		joint_names = ["s0", "s1", "e0", "e1","w0", "w1", "w2"]
		throw = [0,0,0,0,0, -speed,0]
		zero = [0,0,0,0,0,0,0]
		self.throw_dict = {}
		self.zero_dict = {}

		rospy.Subscriber("/robot/limb/"+limb+"/endpoint_state", EndpointState, self.respondToEndpoint)
		self.hand_pose = Pose()

		for i in xrange(0,7) :
			self.throw_dict[limb+'_'+joint_names[i]] = throw[i]
			self.zero_dict[limb+'_'+joint_names[i]] = 0

		self.throw(limb, 1.2, .6)

		#rospy.spin()

	def respondToEndpoint(self, EndpointState) :
		self.hand_pose = deepcopy(EndpointState.pose)


	def throw(self, limb, swing_time, release_time) :
		arm = Limb(limb)

		current_joints = arm.joint_angles()
		#deepcopy(current_joints)
		current_joints[limb+'_w1'] = 2
		arm.move_to_joint_positions(current_joints)
		rospy.sleep(2)
		#move arm all the way back!



		currenttime = rospy.get_time()
		endswing = currenttime + swing_time
		releasetime = currenttime + release_time
		released = False
		rate = rospy.Rate(50)
		oldz = 10000000
		while endswing > rospy.get_time() :
			#releases when hand starts going up again
			#print "oldz, posz"
			print oldz
			#print self.hand_pose.position.z
			if releasetime < rospy.get_time() and not released : 
				print "Beginning gripper open"
				self.gripper.command_position( 100, block=False)
				released = True
				#rospy.sleep(.05)
				print "gripper should be open"
			arm.set_joint_velocities(self.throw_dict)
			oldz = self.hand_pose.position.z
			#print oldz
			rate.sleep()



		#rospy.sleep(.01)
		#self.gripper.command_position( 100, block=False)
		#rospy.sleep(2)
		
		#arm.set_joint_velocities(self.zero_dict)

		
		

if __name__ == '__main__':
	try:
		torque_thrower()
	except rospy.ROSInterruptException:
		pass

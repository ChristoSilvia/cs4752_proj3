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
import random
from game_server.msg import *
from threading import Timer
import time
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

# Joint Range Table
# Joint	Min limit	Max limit	Range (Radians)
# S0	-2.461		+0.890		3.351
# S1	-2.147		+1.047		3.194
# E0	-3.028		+3.028		6.056
# E1	-0.052		+2.618		2.67
# W0	-3.059		+3.059		6.117
# W1	-1.571		+2.094		3.665
# W2	-3.059		+3.059		6.117

#returns ordered list of throws. 0 is most inner, length is outer
def get_ordered_uppers() :
	uppers = []

	#veers right, misses!
	pose = {'w0': -2.3573449730896, 'w1': -0.13000487162475588, 'w2': 0.022626216595458985, 'e0': -2.771136290148926, 'e1': 1.0151117852233886, 's0': -0.08743690480957032, 's1': -0.5783107563720703}
	uppers.append(pose)

	#veers right
	pose = {'w0': -2.4708595512634277, 'w1': -0.01457281746826172, 'w2': 0.027611654150390626, 'e0': -2.625791610662842, 'e1': 0.5426457030944825, 's0': -0.08858739039916992, 's1': -0.796519523199463}
	uppers.append(pose)

	#really high mid to right
	pose = {'w0': -2.2434468997192383, 'w1': 0.06442719301757813, 'w2': 0.046786413977050786, 'e0': -2.7626993958251953, 'e1': 0.9767622655700684, 's0': -0.1587670113647461, 's1': -0.5437961886840821}
	uppers.append(pose)

	#really high release, center A++++ both arms
	pose = {'w0': -2.276810981817627, 'w1': 0.1449611842895508, 'w2': 0.042951462011718754, 'e0': -2.6380634569519046, 'e1': 0.8743690480957031, 's0': -0.2427524594055176, 's1': -0.7075486376037599}
	uppers.append(pose)


	#really high release, veers left.
	pose = {'w0': -2.3339517661010745, 'w1': 0.5161845345336914, 'w2': -0.0065194183410644535, 'e0': -2.7212819145996097, 'e1': 1.136679762524414, 's0': 0.0034514567687988283, 's1': -0.5579855109558106}
	uppers.append(pose)

	return uppers

def get_ordered_middles() :
	middles = []

	pose = {'w0': -3.0590003413140225, 'w1': 0.6707745877077995, 'w2': -0.5000126397492455, 'e0': -1.3369353610858052, 'e1': 1.172494668288068, 's0': -0.07999934123325048, 's1': -1.0000100920451613}
	middles.append(pose)

	pose = {'w0': -2.6461548774463948, 'w1': -0.6628873295811122, 'w2': 0.1619836544514941, 'e0': -2.4273649777016555, 'e1': 0.7579967596882549, 's0': -0.04740248292186955, 's1': -0.7847008492166339}
	middles.append(pose)

	return middles

def get_ordered_banks() :
	banks = []

	pose = {'w0': -3.0590003413140225, 'w1': 0.6707745877077995, 'w2': -0.5000126397492455, 'e0': -1.3369353610858052, 'e1': 1.172494668288068, 's0': -0.07999934123325048, 's1': -1.0000100920451613}
	banks.append(pose)

	return banks


#chooses a throw based on past choices and corresponding score updates
class thrower_ai :
	def __init__(self):

		self.throw_positions = {}
		self.throw_positions['uppers'] = get_ordered_uppers()
		self.throw_positions['middles'] = get_ordered_middles()
		self.throw_positions['banks'] = get_ordered_banks() #two left, then two right


		self.bound = {}
		self.bound['uppers'] = [0, len(self.throw_positions['uppers']) - 1]
		self.bound['middles'] = [0, len(self.throw_positions['middles']) - 1]
		self.bound['banks'] = [0, len(self.throw_positions['banks']) - 1]

		self.current_category = 'uppers'
		self.throw_state = {}
		self.throw_state['uppers'] = 0
		self.throw_state['middles'] = 0
		self.throw_state['banks'] = 0


		self.net_gains = {}
		self.net_gains['uppers'] = 0
		self.net_gains['middles'] = 0
		self.net_gains['banks'] = 0

		self.our_score = 0
		self.our_penalties = 0
		self.enemy_score = 0
		self.enemy_penalties = 0

		self.throw_time = 0
		self.score_window = 15

	def score_before_throw(self, our_s, our_p, enemy_s, enemy_p) :
		print "Sending AI Score so we know how it changes after throw"
		print our_s, our_p, enemy_s, enemy_p
		self.our_score = our_s
		self.our_penalties = our_p
		self.enemy_score = enemy_s
		self.enemy_penalties = enemy_p
		#self.throw_time = throwtime
	

	#call this, so robot can update throw, either keeping current, or adjusting strategy
	def score_after_throw(self, our_s, our_p, enemy_s, enemy_p) :

		print "Score after Throw received in AI"

		gain = (our_s - self.our_score) + (enemy_p - self.enemy_penalties)
		loss = (enemy_s - self.enemy_score) + (our_p - self.our_penalties)
		net_gain = gain - loss
		self.net_gains[self.current_category] += net_gain

		print "Net gain: %d" % net_gain
		print "using index : %d from category : %s" % (self.throw_state[self.current_category], self.current_category)

		if net_gain > 0 :
			print ""
			print "Scored or gave the opponent a penalty same thinG!!!!!!"
			print ""

		if net_gain < 0 :
			print ""
			print "That throw was SHIT or they scored"
			print ""

		#change the throw of this state, not worth trying again...
		if net_gain <= 0 :
			self.throw_state[self.current_category] = (self.throw_state[self.current_category] + 1) % 3
			
			#uppers have failed twice, we need to change
			if self.current_category == 'uppers' and self.net_gains['uppers'] <= -2 :
				print "Uppers failed twice"

				if self.net_gains['middles'] >= self.net_gains['uppers'] or self.net_gains['banks'] >= self.net_gains['middles'] :
					if self.net_gains['middles'] >= self.net_gains['banks'] :
						self.current_category = 'middles'
						print "Switching from Uppers to Middles"
					else :
						self.current_category = 'banks'
						print "Switching from Uppers to Banks"

			elif self.current_category == 'middles' :
				if self.net_gains['uppers'] >= self.net_gains['middles'] or self.net_gains['banks'] >= self.net_gains['middles'] :
					if self.net_gains['uppers'] >= self.net_gains['banks'] :
						self.current_category = 'uppers'
						print "Switching from Middles to uppers"
					else :
						self.current_category = 'banks'
						print "Switching from Middles to banks"

			elif self.current_category == 'banks' :
				if self.net_gains['uppers'] >= self.net_gains['banks'] or self.net_gains['middles'] >= self.net_gains['banks'] :
					if self.net_gains['uppers'] >= self.net_gains['middles'] :
						self.current_category = 'uppers'
						print "Switching from banks to uppers"
					else :
						self.current_category = 'middles'
						print "Switching from banks to middles"


	def get_throw_pose(self) :
		state = self.throw_state[self.current_category] 
		innerIndex = self.bound[self.current_category][0]
		outerIndex = self.bound[self.current_category][1]
		if state == 0 :
			index = innerIndex
		elif state == 1 :
			index = outerIndex
		elif state == 2 :
			index = random.randint(innerIndex+1, outerIndex-1) 
		return (self.throw_positions[self.current_category])[index]
	


class thrower :
	def __init__(self):
		rospy.init_node('thrower')
		rospy.loginfo("Initialized Thrower")

		rs = baxter_interface.RobotEnable(CHECK_VERSION)
		init_state = rs.state().enabled

		self.move_robot = createServiceProxy("move_robot", MoveRobot, "")

		self.canceled = False

		self.game_state = GameState()
		self.game_state.score = [0,0]
		self.game_state.penalty = [0,0]
		self.score_sub = rospy.Subscriber( "/game_server/game_state", GameState, self.update_scores)

		# self.limb = 'left'
		self.limb = rospy.get_param("limb")
		self.gripper = baxter_interface.Gripper(self.limb, CHECK_VERSION)

		self.arm = Limb(self.limb)

		self.arm_kin = baxter_pykdl.baxter_kinematics(self.limb)

		self.target_pose_pub = rospy.Publisher('zic_target_pose', PoseStamped, queue_size=1)

		throw_service = createService('throw', Action, self.throw_srv, "")


		self.ai = thrower_ai()

		rapidFireTesting = False

		rate = rospy.Rate(.09)
		while not rospy.is_shutdown() :


			print "-----------CURRENT ARM POSITIONS------------"
			print self.arm.joint_angles()


			#use this code to test new arm positions and recently recorded ones
			if rapidFireTesting:
				if False:
					self.testLeftArmInitialPositions(45)
				else :
					self.gripper.command_position(100, block=True)
					rospy.sleep(4)
					self.gripper.command_position(0, block=True)

			
			# self.arm.move_to_joint_positions(new_pose)

			# throw the ball
			# self.throw()

			req = ActionRequest()
			resp = self.throw_srv(req)
			print resp

			rate.sleep()
	

		#rospy.spin()

	def update_scores(self, msg) :
		self.game_state = deepcopy(msg)


	def send_score_before_throw(self) :
		gs = self.game_state
		our_team = 1
		enemy_team = 0
		if self.limb == 'left' :
			our_team = 0
			enemy_team = 1
		if gs != None :
			self.ai.score_before_throw(gs.score[our_team], gs.penalty[our_team], gs.score[enemy_team], gs.penalty[enemy_team])
	
	def send_score_after_throw(self) :
		print "In thrower!!!!! send score after throw"
		gs = self.game_state
		our_team = 1
		enemy_team = 0
		if self.limb == 'left' :
			our_team = 0
			enemy_team = 1
		if gs != None :
			self.ai.score_after_throw(gs.score[our_team], gs.penalty[our_team], gs.score[enemy_team], gs.penalty[enemy_team])
	


	def throw_srv(self, req):
		if self.canceled:
			return ActionResponse(False)
		
		if self.canceled:
			return ActionResponse(False)

		self.send_score_before_throw()
		new_pose = self.ai.get_throw_pose()
		
		if self.limb == "right" :
			new_pose = mirror_left_arm_joints(new_pose)

		print "Pose from ai::"
		print new_pose
		# self.arm.move_to_joint_positions(new_pose)
		
		# self.throw()

		#start coroutine to ping score after 10 seconds
		Timer(10, self.send_score_after_throw, ()).start()

		return ActionResponse(True)


	#give an integer from the set and move to that location. these are known hot spots...
	def testLeftArmInitialPositions(self, index) :
		if index == 1:
			new_pose = {'w0': -3.0590003413140225, 'w1': 0.6707745877077995, 'w2': -0.5000126397492455, 
			'e0': -1.3369353610858052, 'e1': 1.172494668288068, 's0': -0.07999934123325048, 's1': 
			-1.0000100920451613}
		elif index == 2 :
			new_pose = {'w0': -2.6461548774463948, 'w1': -0.6628873295811122, 'w2': 0.1619836544514941, 'e0': 
			-2.4273649777016555, 'e1': 0.7579967596882549, 's0': -0.04740248292186955, 's1': -0.7847008492166339}
			
		elif index == 3 :
			new_pose = {'w0': -2.5017333910662423, 'w1': 1.160111690287775, 'w2': 0.0414804259901862, 'e0': 
			-2.3290057728286806, 'e1': 2.054243975896581, 's0': 0.21496177433691788, 's1': -0.26548120061751224}
		elif index == 40 : #really high release, veers left.
			new_pose = {'w0': -2.3339517661010745, 'w1': 0.5161845345336914, 'w2': -0.0065194183410644535, 
			'e0': -2.7212819145996097, 'e1': 1.136679762524414, 
			's0': 0.0034514567687988283, 's1': -0.5579855109558106}
		elif index == 41 : #really high release, center A++++ both arms
			new_pose = {'w0': -2.276810981817627, 'w1': 0.1449611842895508, 'w2': 0.042951462011718754, 
			'e0': -2.6380634569519046, 'e1': 0.8743690480957031, 's0': -0.2427524594055176, 
			's1': -0.7075486376037599}
		
		elif index == 43 : #really high mid to right
			new_pose = {'w0': -2.2434468997192383, 'w1': 0.06442719301757813, 'w2': 0.046786413977050786, 'e0': -2.7626993958251953, 'e1': 0.9767622655700684, 's0': -0.1587670113647461, 's1': -0.5437961886840821}
		elif index == 44 : #tested once, centered
			new_pose = {'w0': -2.299053703216553, 'w1': 0.46786413977050784, 'w2': 0.00038349519653320315, 'e0': -2.6507187984375, 'e1': 1.231019580871582, 's0': -0.21437381486206056, 's1': -0.2918398445617676}
		elif index == 45 : #veers right
			new_pose = {'w0': -2.4708595512634277, 'w1': -0.01457281746826172, 'w2': 0.027611654150390626, 'e0': -2.625791610662842, 'e1': 0.5426457030944825, 's0': -0.08858739039916992, 's1': -0.796519523199463}



		if self.limb == "right" :
			new_pose = mirror_left_arm_joints(new_pose)

		extra_new_pose = {}
		for p in new_pose:
			extra_new_pose[self.limb+'_'+p] = new_pose[p]
		new_pose = extra_new_pose



		self.gripper.command_position(100, block=True)
		self.arm.move_to_joint_positions(new_pose)
		rospy.sleep(1.5)
		self.gripper.command_position(0, block=True)


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

	

	#throws through w1, setting the release to be the initial position
	def throw(self) :
		# test throw params
		#release_pos = Point(0.573, 0.081, 0.036)
		release_vel = Vector3(100.0, 0.00, 0.00)
		
		safty_buffer = math.radians(10.0)
		w1_min = -1.571
		w1_max = 2.094

		linear_speed = np.linalg.norm(vector3_to_numpy(release_vel))
		print "linear_speed (m/sec)"
		print linear_speed

		link_len = 0.31 # need to measure
		angular_speed = linear_speed/link_len
		if angular_speed > 4.0:
			angular_speed = 4.0
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

		back_swing = math.radians(70.0)
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

			# current_speed = self.arm.joint_velocities()[self.limb+'_w1']
			# open_offset = open_gripper_time * current_speed * 1.5
			# open_angle = release_angle + open_offset



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

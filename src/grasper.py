#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import time
from cs4752_proj3.srv import *
from cs4752_proj3.msg import *
from std_msgs.msg import String, Header, Int32
from geometry_msgs.msg import Pose, Quaternion, Point, PoseArray, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import *
import numpy as np
import math
import tf
from tf.transformations import *
from config import *
from baxter_core_msgs.msg import *
from baxter_core_msgs.srv import *
from baxter_interface import *
from baxter_pykdl import baxter_kinematics
import baxter_interface

class grasper:

	def __init__(self):
		print "Initializing Grasper"

		rs = baxter_interface.RobotEnable(CHECK_VERSION)
		init_state = rs.state().enabled

		self.limb_name = rospy.get_param("limb")

		self.gripper = Gripper(self.limb_name)

		self.limb = Limb(self.limb_name)

		self.objs = {}
		self.objs['pink'] = None
		self.objs['blue'] = None

		self.transform_listener = tf.TransformListener()

		self.move_robot = createServiceProxy("move_robot", MoveRobot, "")

		self.cancel_sub = rospy.Subscriber("/action_cancel", Int32, self.cancel, queue_size=1)

		self.blue_sub = rospy.Subscriber("/found_blue_hand", ScreenObj, self.blue_callback, queue_size=1)
		self.pink_sub = rospy.Subscriber("/found_pink_hand", ScreenObj, self.pink_callback, queue_size=1)
		
		grasp_service = createService('grasp', Action, self.grasp, "")

		self.execute_pub = rospy.Publisher("/execute_vel", Vector3)

		self.rate = rospy.Rate(30)

		

		print "done initializing"

		# self.grasp("")

	def cancel(self, msg):
		if msg.data == GRAB:
			self.canceled = True

	def get_position(self):
		return np.array(self.limb.endpoint_pose()['position'])

	def pink_callback(self, data):
		self.objs["pink"] = data

	def blue_callback(self, data):
		self.objs["blue"] = data

	def grasp(self, req):
		color = req.arg
		# color = 'pink'
		grasped = False
		self.canceled = False
		# self.move_robot(MOVE_TO_POSE, self.limb_name, req.pose)
		self.move_robot(OPEN_GRIPPER, self.limb_name, Pose())

		grasp_z = -0.072

		while not grasped and not rospy.is_shutdown():

			if self.canceled:
				return ActionResponse(False)

			max_speed = .15
			max_error = 330
			max_distance = 1.0

			try:
				bi = self.objs[color]
			except:
				bi = None

			if bi != None and bi.radius != 0 :

				actual_screen_coords = np.array([bi.x,bi.y])

				offset_x = 30.0
				offset_y = -140.0
				if color == 'blue':
					offset_y = -120.0
					offset_x = 30.0

				desired_screen_coords = np.array([
					bi.img_height/2. + offset_x,
					bi.img_width/2. + offset_y
				])

				error_screen =  actual_screen_coords - desired_screen_coords

				radius = bi.radius
				pixel_radius_at_meter = 8.4
				distance = pixel_radius_at_meter / radius

				scaled_error = error_screen/max_error * max_speed

				approach_speed = distance / max_distance * max_speed
				approach_speed *= 1 - np.linalg.norm(error_screen)/max_error

				scaled_error = np.append(scaled_error, approach_speed)

				hand_frame = self.limb_name+"_hand_camera"

				v_stamped = Vector3Stamped()
				v_stamped.header.frame_id = hand_frame
				v_stamped.header.stamp = self.transform_listener.getLatestCommonTime(hand_frame, 'base')
				v_stamped.vector = numpy_to_vector3(scaled_error)

				base_vel = self.transform_listener.transformVector3('base', v_stamped)

				if self.get_position()[2] < grasp_z:
					self.move_robot(CLOSE_GRIPPER, self.limb_name, Pose())
					grasped = True
					break
				elif self.limb.endpoint_effort()['force'].z < -6.0:
					up_vec = Vector3(0,0,max_speed*.5)
					self.moveVel(up_vec)
				else:
					self.moveVel(base_vel.vector)

			else:
				# move upwards if ball not found
				if self.get_position()[2] < .55:
					up_vec = Vector3(0,0,max_speed*.5)
					self.moveVel(up_vec)

			self.rate.sleep()

		success = self.confirmGrasp()
		print "success"
		print success
		if not success:
			self.moveVel(up_vec)
			self.moveVel(up_vec)
			self.grasp(req)

		cv2.destroyAllWindows()

		return ActionResponse(success)

	def confirmGrasp(self):
		return self.gripper.gripping()

	def moveVel(self, v):
		self.execute_pub.publish(v)

def main(args):
  
	rospy.init_node('grasper', anonymous=True)
	ic = grasper()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)

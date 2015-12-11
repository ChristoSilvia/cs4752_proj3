#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import time
from cs4752_proj3.srv import *
from std_msgs.msg import String, Header
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
from cs4752_proj3.srv import *

class grasper:

	def __init__(self):
		print "Initializing Grasper"

		rs = baxter_interface.RobotEnable(CHECK_VERSION)
		init_state = rs.state().enabled

		try:
			self.limb_name = rospy.get_param("limb")
		except:
			self.limb_name = 'right'

		self.limb = Limb(self.limb_name)

		self.transform_listener = tf.TransformListener()

		self.move_robot = createServiceProxy("move_robot", MoveRobot, "")

		self.blue_sub = rospy.Subscriber("/found_blue_hand", Vector3, self.blue_callback, queue_size=1)
		self.pink_sub = rospy.Subscriber("/found_pink_hand", Vector3, self.pink_callback, queue_size=1)
		
		# self.ball_pub = rospy.Publisher("/ball_pose_hand",PoseStamped)

		self.execute_pub = rospy.Publisher("/execute_vel", Vector3)

		self.rate = rospy.Rate(30)

		self.objs = {}
		self.objs['pink'] = None
		self.objs['blue'] = None

		self.rgb_image_shape = (600, 960, 3)


		print "done initializing"

		self.grasp("blue")
		# self.grasp("pink")

	def get_position(self):
		return np.array(self.limb.endpoint_pose()['position'])

	def pink_callback(self, data):
		self.objs["pink"] = data

	def blue_callback(self, data):
		self.objs["blue"] = data

	def grasp(self, color) :

		grasped = False
		self.move_robot(OPEN_GRIPPER, self.limb_name, Pose())

		grasp_z = -0.075

		while not grasped and not rospy.is_shutdown():

			max_speed = .12
			max_error = 330
			max_distance = 1.0

			try:
				bi = self.objs[color]
			except:
				bi = None

			if bi != None and bi.z != 0 :

				actual_screen_coords = np.array([bi.x,bi.y])

				offset_x = 0.0
				offset_y = 0.0
				if color == 'blue':
					offset_y = -120.0

				desired_screen_coords = np.array([
					self.rgb_image_shape[1]/2. + offset_x,
					self.rgb_image_shape[0]/2. + offset_y
				])

				error_screen =  actual_screen_coords - desired_screen_coords

				radius = bi.z
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
				else:
					self.moveVel(base_vel.vector)

			else:
				# move upwards if ball not found
				if self.get_position()[2] < .6:
					up_vec = Vector3(0,0,max_speed*.5)
					self.moveVel(up_vec)

			self.rate.sleep()

		cv2.destroyAllWindows()

	def moveVel(self, v):
		# pass
		# print v
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

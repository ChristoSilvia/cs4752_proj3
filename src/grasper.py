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

class HSVMask:
	def __init__(self, hue_min, hue_max, sat_min, sat_max, val_min, val_max):
		self.m = {}
		self.m["H"] = {}
		self.m["H"]["min"] = hue_min
		self.m["H"]["max"] = hue_max
		self.m["S"] = {}
		self.m["S"]["min"] = sat_min
		self.m["S"]["max"] = sat_max
		self.m["V"] = {}
		self.m["V"]["min"] = val_min
		self.m["V"]["max"] = val_max

	def changeMask(self, param, arg, inc):
		limit = {}
		limit["H"] = {}
		limit["S"] = {}
		limit["V"] = {}
		limit["H"]["min"] = 0
		limit["S"]["min"] = 0
		limit["V"]["min"] = 0
		limit["H"]["max"] = 180
		limit["S"]["max"] = 255
		limit["V"]["max"] = 255

		self.m[param][arg] += inc

		if arg == "min":
			if self.m[param][arg] < limit[param][arg]:
				self.m[param][arg] = limit[param][arg]
			elif self.m[param][arg] > self.m[param]["max"]:
				self.m[param][arg] = self.m[param]["max"]
		elif arg == "max":
			if self.m[param][arg] > limit[param][arg]:
				self.m[param][arg] = limit[param][arg]
			elif self.m[param][arg] < self.m[param]["min"]:
				self.m[param][arg] = self.m[param]["min"]
		
		print "%s %s" % (param, arg)
		print self.m[param][arg]

class grasper:

	def __init__(self):
		print "Initializing Grasper"

		rs = baxter_interface.RobotEnable(CHECK_VERSION)
		init_state = rs.state().enabled

		# try:
		# 	self.limb_name = rospy.get_param("limb")
		# except:
		self.limb_name = 'right'

		self.limb = Limb(self.limb_name)

		self.lastImageTime = time.time()
		self.imageWaitTime = .01

		self.pink_mask = HSVMask(
			170.0, 182.0,
			70.0, 255.0,
			70.0, 255.0
		)
		self.blue_mask = HSVMask(
			100.0, 140.0,
			60.0, 220.0,
			00.0, 180.0
		)

		self.calibrated = True # set to false when you need to calibrate
		self.param = "H"
		self.prompt = True

		self.bridge = CvBridge()	

		self.transform_listener = tf.TransformListener()

		self.move_robot = createServiceProxy("move_robot", MoveRobot, "")

		topic = "/cameras/"+self.limb_name+"_hand_camera/image"
		self.image_sub = rospy.Subscriber(topic,Image,self.imagecallback, queue_size=1)
		self.ball_pub = rospy.Publisher("/ball_pose_hand",PoseStamped)

		self.execute_pub = rospy.Publisher("/execute_vel", Vector3)

		self.rate = rospy.Rate(30)

		self.rgb_image = None
		
		print "subscribed to %s" % topic
		print "done initializing"

		self.window_name = 'Grasping'
		cv2.startWindowThread()
		cv2.namedWindow(self.window_name)

		# self.grasp(self.pink_mask)
		self.grasp(self.blue_mask)

	def get_position(self):
		return np.array(self.limb.endpoint_pose()['position'])

	def imagecallback(self,data):
		try:
			self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")#rgba8
		except CvBridgeError, e:
			print e

	def findBlobsofHue(self, hsv_mask, num_blobs, rgbimage) :

		colorLower = (
			hsv_mask.m["H"]["min"], 
			hsv_mask.m["S"]["min"], 
			hsv_mask.m["V"]["min"]
		)
		colorUpper = (
			hsv_mask.m["H"]["max"], 
			hsv_mask.m["S"]["max"], 
			hsv_mask.m["V"]["max"]
		)

		blurred = cv2.GaussianBlur(rgbimage, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, colorLower, colorUpper)

		res = cv2.bitwise_and(rgbimage,rgbimage, mask = mask)

		blobsFound = []
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]

		while num_blobs > 0 and len(cnts) > 0:
			c = max(cnts, key=cv2.contourArea)
			
			((x, y), radius) = cv2.minEnclosingCircle(c)
	 		num_blobs = num_blobs - 1
			
			if radius > 7:# only proceed if the radius meets a minimum size
				blobsFound.append([x,y,radius])
				cv2.circle(res, (int(x), int(y)), int(radius), (0,255,255), 2)

		cv2.imshow("Grasping",res)

		if not self.calibrated:
			self.calibrate(hsv_mask)

		return blobsFound


	def calibrate(self, hsv_mask):
		if self.prompt :
			print "Calibrating HSV Mask"
			print "Click on the image, then..."
			print "use H, S, and V to switch the changing param"
			print ""
			print "use U and J to change min"
			print "use O and L to change max"
			print ""
			print "press any other key to get a new frame"
			print "Press space when done"
			self.prompt = False

		key = cv2.waitKey(0) & 0xFF

		# print key

		# # if the space key is pressed, add the point to the dict
		if key == ord("u"):
			hsv_mask.changeMask(self.param, "min", .5)

		if key == ord("j"):
			hsv_mask.changeMask(self.param, "min", -.5)
		
		if key == ord("o"):
			hsv_mask.changeMask(self.param, "max", .5)
		
		if key == ord("l"):
			hsv_mask.changeMask(self.param, "max", -.5)


		if key == ord("h"):
			print "Now altering H"
			self.param = "H"

		if key == ord("s"):
			print "Now altering S"
			self.param = "S"
		
		if key == ord("v"):
			print "Now altering V"
			self.param = "V"


		if key == ord(" "):
			self.calibrated = True
			print "############## Calibrated Mask ##############"
			print hsv_mask.m
			print "#############################################"
			
	def grasp(self, HSVMask) :

		grasped = False
		self.move_robot(OPEN_GRIPPER, self.limb_name, Pose())

		grasp_z = -0.07

		while not grasped or not rospy.is_shutdown():

			obj = self.findBlobsofHue(HSVMask, 1, self.rgb_image)

			max_speed = .12
			max_error = 330
			max_distance = 1.0

			if obj != [] :
				bi = obj[0]

				actual_screen_coords = np.array(bi[:2])

				offset_x = 0.0
				offset_y = -120.0
				desired_screen_coords = np.array([
					self.rgb_image.shape[1]/2. + offset_x,
					self.rgb_image.shape[0]/2. + offset_y
				])

				error_screen =  actual_screen_coords - desired_screen_coords

				radius = bi[2]
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
		if self.calibrated:
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
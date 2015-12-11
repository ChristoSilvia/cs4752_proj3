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
	def __init__(self, name, mask, calibrated=True):
		self.name = name

		self.calibrated = calibrated # set to false when you need to calibrate
		self.param = "H"
		self.prompt = True
		self.m = mask

	def changeMask(self, param, arg, inc):
		limit = {}
		limit["H"] = {}
		limit["S"] = {}
		limit["V"] = {}
		limit["D"] = {}
		limit["H"]["min"] = 0
		limit["S"]["min"] = 0
		limit["V"]["min"] = 0
		limit["D"]["min"] = -1.0
		limit["H"]["max"] = 180
		limit["S"]["max"] = 255
		limit["V"]["max"] = 255
		limit["D"]["max"] = 3.0

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

	def calibrate(self):
		if self.prompt :
			print "Calibrating %s HSV Mask" % self.name
			print "Click on the image, then..."
			print "use H, S, V, and D to switch the changing param"
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

		inc = .5
		if self.param == "D":
			inc = .05

		if key == ord("u"):
			self.changeMask(self.param, "min", inc)

		if key == ord("j"):
			self.changeMask(self.param, "min", -inc)
		
		if key == ord("o"):
			self.changeMask(self.param, "max", inc)
		
		if key == ord("l"):
			self.changeMask(self.param, "max", -inc)


		if key == ord("h"):
			print "Now altering H"
			self.param = "H"

		if key == ord("s"):
			print "Now altering S"
			self.param = "S"
		
		if key == ord("v"):
			print "Now altering V"
			self.param = "V"

		if key == ord("d"):
			print "Now altering D"
			self.param = "D"


		if key == ord(" "):
			self.calibrated = True
			print "############## Calibrated Mask ##############"
			print self.m
			print "#############################################"

class Vision:

	def __init__(self):
		self.vision_type = rospy.get_param("vision_type")

		rospy.init_node("%s_vision" % self.vision_type)

		print "Initializing %s Vision" % self.vision_type

		try:
			self.limb_name = rospy.get_param("limb")
		except:
			self.limb_name = 'right'

		self.lastImageTime = time.time()
		self.imageWaitTime = .01

		self.pink_kinect_mask = HSVMask(
			"pink",
			{'H': {'max': 168.0, 'min': 154.5}, 'S': {'max': 255.0, 'min': 70.0}, 'D': {'max': 1.7, 'min': 1.4}, 'V': {'max': 255.0, 'min': 70.0}},
			calibrated=True
		)
		self.pink_hand_mask = HSVMask(
			"pink",
			{'H': {'max': 180.0, 'min': 169.0}, 'S': {'max': 255.0, 'min': 100.0}, 'D': {'max': 1.7, 'min': 1.4}, 'V': {'max': 255, 'min': 20.0}},
			calibrated=True
		)

		self.blue_kinect_mask = HSVMask(
			"blue",
			{'H': {'max': 140.0, 'min': 100.0}, 'S': {'max': 220.0, 'min': 60.0}, 'D': {'max': 1.75, 'min': 1.4}, 'V': {'max': 180.0, 'min': 0.0}},
			calibrated=True
		)
		self.blue_hand_mask = HSVMask(
			"blue",
			{'H': {'max': 140.0, 'min': 100.0}, 'S': {'max': 220.0, 'min': 60.0}, 'D': {'max': 1.75, 'min': 1.4}, 'V': {'max': 180.0, 'min': 0.0}},
			calibrated=True
		)



		self.bridge = CvBridge()	

		if self.vision_type == "kinect":
			self.rgb_topic = "/camera/rgb/image_rect_color"

			self.depth_topic ="/camera/depth_registered/hw_registered/image_rect"
			self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=1)
			print "subscribed to %s" % self.depth_topic
			self.depth_image = None

		elif self.vision_type == "hand":
			self.rgb_topic = "/cameras/"+self.limb_name+"_hand_camera/image"

		self.rgb_sub = rospy.Subscriber(self.rgb_topic,Image,self.rgb_callback, queue_size=1)
		print "subscribed to %s" % self.rgb_topic
		self.rgb_image = None

		self.pink_pub = rospy.Publisher("/found_pink_%s" % self.vision_type, Vector3, queue_size=1)
		self.blue_pub = rospy.Publisher("/found_blue_%s" % self.vision_type, Vector3, queue_size=1)
		
		self.rate = rospy.Rate(30)
		
		print "done initializing"

		cv2.startWindowThread()
		cv2.namedWindow('%s %s vision' % ("pink", self.vision_type))

		cv2.startWindowThread()
		cv2.namedWindow('%s %s vision' % ("blue", self.vision_type))

		rospy.spin()

	def rgb_callback(self,data):
		try:
			self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")#rgba8
		except CvBridgeError, e:
			print e

		pink_mask = None
		blue_mask = None
		if self.vision_type == "kinect":
			pink_mask = self.pink_kinect_mask
			blue_mask = self.blue_kinect_mask
		elif self.vision_type == "hand":
			pink_mask = self.pink_hand_mask
			blue_mask = self.blue_hand_mask
		
		pinks = self.findBlobsofHue(pink_mask, 1 , self.rgb_image)
		if len(pinks) > 0:
			self.pink_pub.publish(Vector3(pinks[0][0],pinks[0][1],pinks[0][2]))
		else:
			self.pink_pub.publish(Vector3(0,0,0))

		blues = self.findBlobsofHue(blue_mask, 1 , self.rgb_image)
		if len(blues) > 0:
			self.blue_pub.publish(Vector3(blues[0][0],blues[0][1],blues[0][2]))
		else:
			self.blue_pub.publish(Vector3(0,0,0))

	def depth_callback(self,data):
		# print "got depth"
		try:
			self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
		except CvBridgeError, e:
			print e

	def findBlobsofHue(self, hsv_mask, num_blobs, rgbimage) :
		if self.vision_type == 'kinect' and self.depth_image == None:
			return []

		# if hsv_mask_name == 'pink':
		# 	hsv_mask = self.pink_mask
		# elif hsv_mask_name == 'blue'
		# 	hsv_mask = self.blue_mask

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

		if self.vision_type == "kinect":
			mask = cv2.inRange(self.depth_image, hsv_mask.m["D"]["min"], hsv_mask.m["D"]["max"])
			rgbimage = cv2.bitwise_and(rgbimage,rgbimage,mask = mask)

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

		window_name = '%s %s vision' % (hsv_mask.name, self.vision_type)
		# print window_name
		cv2.imshow(window_name, res)

		if not hsv_mask.calibrated:
			hsv_mask.calibrate()

		return blobsFound

if __name__ == '__main__':
	try:
		v = Vision()
	except KeyboardInterrupt:
		print "Shutting down"
	cv2.destroyAllWindows()
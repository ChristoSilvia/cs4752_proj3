#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import time
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
from cs4752_proj3.msg import *
from cs4752_proj3.srv import *
import sys, os

class HSVMask:
	def __init__(self, color, camera, mask, num_blobs=1, calibrated=True):
		self.color = color
		self.camera = camera

		self.calibrated = calibrated # set to false when you need to calibrate
		self.param = "H"
		self.prompt = True
		self.m = mask
		self.num_blobs = num_blobs

		if self.color == 'blue':
			self.shape = cv2.imread('../ros_ws/src/cs4752_proj3/img/square.jpg', 0)
		elif self.color == 'pink':
			self.shape = cv2.imread('../ros_ws/src/cs4752_proj3/img/circle.jpg', 0)

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
			print "Calibrating %s %s HSV Mask" % (self.color, self.camera)
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
			print "############## Calibrated %s %s Mask ##############" % (self.color, self.camera)
			print self.m
			print "#############################################"

class Vision:

	def __init__(self):

		self.vision_type = rospy.get_namespace()[1:-1]
		# self.vision_type = 'hand'

		rospy.init_node("%s_vision" % self.vision_type)

		print "Initializing %s Vision" % self.vision_type

		self.limb_name = rospy.get_param("/limb")
		self.num_blocks = rospy.get_param("/num_blocks")

		# rospy.set_param("/camera/driver/depth_registration", True)

		self.lastImageTime = time.time()
		self.imageWaitTime = .01

		self.pink_kinect_mask = HSVMask(
			"pink",
			"kinect",
			{'H': {'max': 168.0, 'min': 154.5}, 'S': {'max': 255.0, 'min': 70.0}, 'D': {'max': 1.7, 'min': 1.4}, 'V': {'max': 255.0, 'min': 70.0}},
			calibrated=True
		)
		self.blue_kinect_mask = HSVMask(
			"blue",
			"kinect",
			{'H': {'max': 140.0, 'min': 100.0}, 'S': {'max': 180.0, 'min': 83.0}, 'D': {'max': 1.7, 'min': 1.4}, 'V': {'max': 255, 'min': 115.0}},
#			calibrated=False,
			calibrated=True,
			num_blobs=self.num_blocks
		)

		self.pink_hand_mask = HSVMask(
			"pink",
			"hand",
			{'H': {'max': 180.0, 'min': 169.0}, 'S': {'max': 255.0, 'min': 100.0}, 'V': {'max': 255, 'min': 20.0}},
			calibrated=True
		)
		self.blue_hand_mask = HSVMask(
			"blue",
			"hand",
			{'H': {'max': 140.0, 'min': 100.0}, 'S': {'max': 220.0, 'min': 50.0}, 'V': {'max': 252.0, 'min': 50.0}},
			calibrated=True
		)


		self.transform_listener = tf.TransformListener()
		self.bridge = CvBridge()	

		if self.vision_type == "kinect":
			self.rgb_topic = "/camera/rgb/image_rect_color"

			self.depth_topic ="/camera/depth_registered/hw_registered/image_rect"
			#self.depth_topic = "/camera/depth/image_rect"
			# self.depth_topic = "/camera/depth_registered/sw_registered/image_rect"
			self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=1)
			print "subscribed to %s" % self.depth_topic

		elif self.vision_type == "hand":
			self.rgb_topic = "/cameras/"+self.limb_name+"_hand_camera/image"

		self.rgb_sub = rospy.Subscriber(self.rgb_topic,Image,self.rgb_callback, queue_size=1)
		print "subscribed to %s" % self.rgb_topic

		self.rgb_image = None
		self.depth_image = None

		self.pink_screen_pub = rospy.Publisher("/found_pink_%s" % self.vision_type, ScreenObj, queue_size=1)
		self.blue_screen_pub = rospy.Publisher("/found_blue_%s" % self.vision_type, ScreenObj, queue_size=1)
		
		self.pink_base_pub = rospy.Publisher("/ball_pose_%s" % self.vision_type, PoseStamped, queue_size=1)
		self.blue_base_pub = rospy.Publisher("/block_poses_%s" % self.vision_type, PoseStamped, queue_size=1)
		
		self.rate = rospy.Rate(30)
		self.pixel_radius = 10#2.1539 #radius in pixels at 1 meter of orange ball
		
		if self.vision_type != 'hand':
			cv2.startWindowThread()
			cv2.namedWindow('%s %s vision' % ("pink", self.vision_type))
			print '%s %s vision' % ("pink", self.vision_type)

			cv2.startWindowThread()
			cv2.namedWindow('%s %s vision' % ("blue", self.vision_type))

		print "done initializing"

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
		
		self.find_project_publish(pink_mask)
		self.find_project_publish(blue_mask)

	def find_project_publish(self, hsv_mask):
		screen_pub = None
		base_pub = None
		if hsv_mask.color == 'blue':
			screen_pub = self.blue_screen_pub
			base_pub = self.blue_base_pub
		elif hsv_mask.color == 'pink':
			screen_pub = self.pink_screen_pub
			base_pub = self.pink_base_pub

		objs = self.findBlobsofHue(hsv_mask, hsv_mask.num_blobs , self.rgb_image)
		if len(objs) > 0:
			screen_pub.publish(ScreenObj(objs[0][0],objs[0][1],objs[0][2],self.rgb_image.shape[0],self.rgb_image.shape[1]))
		else:	
			screen_pub.publish(ScreenObj(0,0,0,0,0))

		if objs != [] :
			bi = objs[0]
			radius = bi[2]
			if self.depth_image != None :
				distance = self.depth_image[int(bi[1]), int(bi[0])]
				if math.isnan(distance) :
					distance = self.pixel_radius / radius
				obj_pose = self.project((bi[0], bi[1]), distance, self.rgb_image.shape[1], self.rgb_image.shape[0])
				
			else :
				distance = self.pixel_radius / radius
				obj_pose = self.project((bi[0], bi[1]), distance, self.rgb_image.shape[1], self.rgb_image.shape[0])
			try:
				if obj_pose != None :
					image_frame = ""
					if self.vision_type == "kinect":
						image_frame = "/camera_rgb_optical_frame"
					elif self.vision_type == "hand":
						image_frame = "/%s_hand_camera" % self.limb_name
					obj_image_pose = PoseStamped()
					obj_image_pose.header.frame_id = image_frame

					# self.suppress_stderr()
					stamp = self.transform_listener.getLatestCommonTime(image_frame, 'base')
					# self.enable_stderr()
					
					obj_image_pose.header.stamp = stamp
					obj_image_pose.pose = obj_pose

					obj_base_pose = self.transform_listener.transformPose('base', obj_image_pose)
					obj_base_pose.header.stamp = rospy.Time.now()

					base_pub.publish(obj_base_pose)
			except CvBridgeError, e:
				print e

	def depth_callback(self,data):
		try:
			self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
		except CvBridgeError, e:
			print e

	def findBlobsofHue(self, hsv_mask, num_blobs, rgb_image) :
		if self.vision_type == 'kinect' and self.depth_image == None:
			print("Error: no depth image")
			return []

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

		if self.vision_type == "kinect" and self.depth_image != None:
			mask = cv2.inRange(self.depth_image, hsv_mask.m["D"]["min"], hsv_mask.m["D"]["max"])
			self.rgb_image = cv2.bitwise_and(self.rgb_image, rgb_image, mask = mask)

		blurred = cv2.GaussianBlur(self.rgb_image, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, colorLower, colorUpper)

		res = cv2.bitwise_and(self.rgb_image,self.rgb_image, mask = mask)

		blobsFound = []
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]

		h = self.rgb_image.shape[0]
		w = self.rgb_image.shape[1]
		center_img = np.array([w/2.,h/2.])
		ret, thresh = cv2.threshold(hsv_mask.shape, 127, 255,0)
		contours,hierarchy = cv2.findContours(thresh,2,1)
		shape = contours[0]

		def get_score(c):
			try:
				# use shape of the obj to get similarity
				similarity = cv2.matchShapes(shape,c,1,0.0) + 1
				area = cv2.contourArea(c)
				M = cv2.moments(c)
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				centroid = np.array([cx,cy])
				center_error = np.linalg.norm(centroid - center_img)
				score = 1 / (center_error / (area * .25) * (similarity * .1))
				return score
			except:
				return 0

		def score_compare(c1, c2):
			return int(get_score(c1) - get_score(c2))

		cs = sorted(cnts, cmp=score_compare)[-num_blobs:]

		for c in cs:

			area = cv2.contourArea(c)
			similarity = cv2.matchShapes(shape,c,1,0.0) + 1
			# print similarity
			# print area

			if hsv_mask.color == 'blue':

				rect = cv2.minAreaRect(c)
				box = cv2.cv.BoxPoints(rect)
				box = np.int0(box)
				
				radius = math.sqrt(cv2.contourArea(c))
				if radius > 5 and area > 150 and similarity < 10.0:
					# get the centroid
					M = cv2.moments(c)
					cx = int(M['m10']/M['m00'])
					cy = int(M['m01']/M['m00'])

					blobsFound.append([cx,cy,radius])
					cv2.drawContours(res,[box],0,(0,0,255),2)
			
			elif hsv_mask.color == 'pink':
				((x, y), radius) = cv2.minEnclosingCircle(c)
				
				if radius > 3 and area > 80 and similarity < 2.0:
					blobsFound.append([x,y,radius])
					cv2.circle(res, (int(x), int(y)), int(radius), (0,255,255), 2)

		if self.vision_type != 'hand':
			window_name = '%s %s vision' % (hsv_mask.color, hsv_mask.camera)
			cv2.imshow(window_name, res)

		if not hsv_mask.calibrated:
			hsv_mask.calibrate()

		return blobsFound

	#creates an intrinsic camera matrix and uses the 
	#position and size of the ball to determine pose
	#relative to the camera, (using kinect specs)
	def project(self, point, distance, width, height) :
		#print point
		#print width
		#print height
		#print "radius"
		#print radius
		#print "Not using depth"

		xFOV = 63.38
		yFOV = 48.25
		cx = width /2
		cy = height /2
		fx = cx / np.tan((xFOV/2) * np.pi / 180)
		fy = cy / np.tan((yFOV/2) * np.pi / 180)
		
		toball = np.zeros(3)
		toball[0] = (point[0] - cx) / fx
		toball[1] = -(point[1] - cy) / fy
		toball[2] = 1
		toball = toball / np.linalg.norm(toball)
		
		toball = toball * distance

		pose = Pose()
		pose.position = Point(toball[0], -toball[1], toball[2])
		pose.orientation = Quaternion(0,0,0,1)
		return pose

	def suppress_stderr(self):
		self.stderr = sys.stderr
		null = open(os.devnull,'wb')
		sys.stderr = os.devnull

	def enable_stderr(self):
		sys.stderr = self.stderr

if __name__ == '__main__':
	try:
		v = Vision()
	except KeyboardInterrupt:
		print "Shutting down"
	cv2.destroyAllWindows()

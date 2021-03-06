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
import numpy as np
import math
from tf.transformations import *
from config import *


#http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython



class single_color_vision:

	def __init__(self):
		print "initializing single color vision object"
		self.ball_pub = rospy.Publisher("/ball_pose",PoseStamped)
		#self.block_pub = rospy.Publisher("/block_poses", PoseArray)
		
		self.get_block_poses = createService("get_block_poses", BlockPoses, self.findBlocks, "")
	
		self.pixel_radius = 10#2.1539 #radius in pixels at 1 meter of orange ball
		self.lastImageTime = time.time()
		self.imageWaitTime = .01

		self.bluehueVal = 110
		
		self.pinkhueVal = 164
		self.hue_range = 6

		self.depthVal = 1.55
		self.depthRange = .25
			
		self.pink_ball_pubs = 0
		self.calibrated = False
		self.prompt = True
		#160 #pink

		#topic for the raw image/camera/depth_registered/image_raw
		#try camera/rgb/image_color/compressed for greater efficiency
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color",Image,self.imagecallback, queue_size=1)
		self.depth_image_sub = rospy.Subscriber("/camera/depth_registered/hw_registered/image_rect", Image, self.depthcallback, queue_size=1)
		self.depth_image = None
		self.rgb_image = None
		self.depth_image_set = False
		#self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image",Image,self.imagecallback, queue_size=1)
		print "subscribed to /camera/rgb/image_rect_color"
		print "done initializing"

		cv2.startWindowThread()
		cv2.namedWindow('HSV_Mask_PINK_BALL')

		cv2.startWindowThread()
		cv2.namedWindow('HSV_Mask_BLUE_BLOCKS')

		# Some interesting cv2 methods that could be used for segment finding and intersection
		# cvtColor
		# BILATERALfILTER
		# CANNY
		# houghlines
		# arclength
		# drawContors


	#def cameraIntrinsicsCB (self, data) :
	#	self.camera_matrix = data.k

	def imagecallback(self,data):
		#don't need to do every frame
		if self.lastImageTime > time.time() - self.imageWaitTime :
			return

		#print "got image"
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")#rgba8
		except CvBridgeError, e:
			print e
		#cv_image = data.image
		#print "Image shape"
		#print cv_image.shape
		
		#TODO fix depth image

		if self.depth_image != None :
			#print "Got depth"
			depthLower = self.depthVal - self.depthRange
			depthUpper = self.depthVal + self.depthRange
			mask = cv2.inRange(self.depth_image, depthLower, depthUpper)
			cv_image = cv2.bitwise_and(cv_image,cv_image,mask = mask)
			#print "imshowing new code"
			
			#cv2.imshow('HSV_Mask_BLUE_BLOCKS',cv_image)
			self.rgb_image = cv_image
			self.findObjects(cv_image)
			self.lastImageTime = time.time()


	def depthcallback(self,data):
		try:
			self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")#rgba8
		except CvBridgeError, e:
			print e

	def vecOut(self, point, width, height) :
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
		toball = toball / np.linalg.norm(toball) #normaliz


	#creates an intrinsic camera matrix and uses the 
	#position and size of the ball to determine pose
	#relative to the camera, (using kinect specs)
	def project(self, point, radius, width, height) :
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
		toball = toball / np.linalg.norm(toball) #normalize so we can then multiply by distance
		distance = self.pixel_radius / radius
		toball = toball * distance

		pose = Pose()
		pose.position = Point(toball[0], toball[1], toball[2])
		pose.orientation = Quaternion(0,0,0,1)
		#print "FOUND Pink BALL!!!!"
		#print toball
		return pose

	def projectDepth(self, point, distance, width, height) :

		#print distance
		#print point
		#print width
		#print height
		#print "radius"
		#print radius

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
		toball = toball / np.linalg.norm(toball) #normalize so we can then multiply by distance
		#distance = self.pixel_radius / radius
		toball = toball * distance

		pose = Pose()
		pose.position = Point(toball[0], toball[1], toball[2])
		pose.orientation = Quaternion(0,0,0,1)
		#print pose.position
		return pose

	def findBlobsofHue(self, hueVal, lookfor, rgbimage) :

		
		colorLower = (hueVal-self.hue_range, 70, 70)
		colorUpper =(hueVal+self.hue_range, 255, 255)
		blurred = cv2.GaussianBlur(rgbimage, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, colorLower, colorUpper)
		#mask = cv2.erode(mask, None, iterations=2)
		#mask = cv2.dilate(mask, None, iterations=2)

		res = cv2.bitwise_and(rgbimage,rgbimage,mask = mask)

		blobsFound = []
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]

		hsvstring = 'HSV_Mask_PINK_BALL'
		if hueVal > 100 and hueVal < 130 :
			hsvstring = 'HSV_Mask_BLUE_BLOCKS'


		while lookfor > 0 and len(cnts) > 0:
			c = max(cnts, key=cv2.contourArea)
			#try :
			#print type(cnts[0])
			#cnts.remove(c) TODO fix how to delete numpy from list
			#except :
				#pass
			
			((x, y), radius) = cv2.minEnclosingCircle(c)
	 		lookfor = lookfor - 1
			
			if radius > 3:# only proceed if the radius meets a minimum size
				blobsFound.append([x,y,radius])
				cv2.circle(res, (int(x), int(y)), int(radius), (0,255,255), 2)

		cv2.imshow(hsvstring,res)

		if not self.calibrated:
			if self.prompt :
				print "Calibrating"
				print "Click on the image, then..."
				print "use W and S to change hue val"
				print "use A and D to change hue range"
				print ""
				print "use I and K to change depth val"
				print "use J and L to change depth range"
				print ""
				print "press any other key to get a new frame"
				print "Press space when done"
				self.prompt = False

			key = cv2.waitKey(0) & 0xFF

			# # if the space key is pressed, add the point to the dict
			pinkmax = 175
			pinkmin = 130
			if key == ord("w"):
				self.pinkhueVal = (self.pinkhueVal + .5) % pinkmax
				if self.pinkhueVal < pinkmin :
					self.pinkhueVal = pinkmin

				print "Pink Hue Val"
				print self.pinkhueVal

			if key == ord("s"):
				self.pinkhueVal = (self.pinkhueVal - .5) % pinkmax
				if self.pinkhueVal < pinkmin :
					self.pinkhueVal = pinkmin

				print "Pink Hue Val"
				print self.pinkhueVal
			
			if key == ord("a"):
				self.hue_range -= .5
				if self.hue_range < 1 :
					self.hue_range = 1

				print "Pink hue_range"
				print self.hue_range
			
			if key == ord("d"):
				self.hue_range += .5
				if self.hue_range < 1 :
					self.hue_range = 1

				print "Pink hue_range"
				print self.hue_range


			if key == ord("i"):
				self.depthVal += .05
				print "depthVal"
				print self.depthVal

			if key == ord("k"):
				self.depthVal -= .05
				print "depthVal"
				print self.depthVal
			
			if key == ord("j"):
				self.depthRange -= .05
				print "depthRange"
				print self.depthRange
			
			if key == ord("l"):
				self.depthRange += .05
				print "depthRange"
				print self.depthRange

			if key == ord(" "):
				self.calibrated = True

		return blobsFound

	def findBlocks(self, req) :
		blockList = self.findBlobsofHue(self.bluehueVal, req.num_blocks, self.rgb_image)
		block_poses_list = []
		for block in blockList :
			if self.depth_image != None :
				distance = self.depth_image[int(block[0]), int(block[1])]
				if not math.isnan(distance) :
					block_pose = self.projectDepth((int(block[0]), int(block[1])), distance, rgbimage.shape[1], rgbimage.shape[0])
				else :
					block_pose = self.project((int(block[0]), int(block[1])), int(block[2]), rgbimage.shape[1], rgbimage.shape[0])
			else :
				block_pose = self.project((int(block[0]), int(block[1])), int(block[2]), rgbimage.shape[1], rgbimage.shape[0])
			block_poses_list.append(block_pose)
		
		

		block_poses = PoseArray()
		block_poses.header = Header()
		#base_pose_stamped.header.seq = self.pink_ball_pubs
		base_pose_stamped.header.stamp = rospy.Time.now()
		base_pose_stamped.header.frame_id = "kinect_frame"
		block_poses.poses = block_poses_list
		return BlockPosesResponse(block_poses)
	
	
		

	#find a ball and return its transform relative to the camera
	#http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
	def findObjects(self, rgbimage) :
		
		initialtime = time.time()

		#used for TUNING
		
		#self.findBlocks()

		pink_balls = self.findBlobsofHue(self.pinkhueVal, 2, rgbimage)
		if pink_balls != [] :
			bi = pink_balls[0]
			if self.depth_image != None :
				distance = self.depth_image[int(bi[1]), int(bi[0])]
				if not math.isnan(distance) :
					ball_pose = self.projectDepth((int(bi[0]), int(bi[1])), distance, rgbimage.shape[1], rgbimage.shape[0])
	 			else :
	 				ball_pose = self.project((int(bi[0]), int(bi[1])), int(bi[2]), rgbimage.shape[1], rgbimage.shape[0])
	 		else :
	 			ball_pose = self.project((int(bi[0]), int(bi[1])), int(bi[2]), rgbimage.shape[1], rgbimage.shape[0])
			try:
				if ball_pose != None :
					self.pink_ball_pubs += 1
					base_pose_stamped = PoseStamped()
					base_pose_stamped.header = Header()
					base_pose_stamped.header.seq = self.pink_ball_pubs
					base_pose_stamped.header.stamp = rospy.Time.now()
					base_pose_stamped.header.frame_id = "kinect_frame"
					base_pose_stamped.pose = ball_pose
					self.ball_pub.publish(base_pose_stamped)
			except CvBridgeError, e:
				print e

		
		


def main(args):
  
  rospy.init_node('single_color_vision', anonymous=True)
  print "initialized node single color vision"
  ic = single_color_vision()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

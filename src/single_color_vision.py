#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import time
from std_msgs.msg import String, Header
from geometry_msgs.msg import Pose, Quaternion, Point, PoseArray
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math



#http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython



class single_color_vision:

	def __init__(self):
		print "initializing single color vision object"
		self.ball_pub = rospy.Publisher("/ball_pose",Pose)
		self.block_pub = rospy.Publisher("/block_poses", PoseArray)
		#self.camerainfo_sub = rospy.Subscriber('camera/rgb/camera_info', CameraInfo, self.cameraIntrinsicsCB)
		#self.camera_matrix = None
		#cv2.namedWindow("Image window", 1)
		self.pixel_radius = 10#2.1539 #radius in pixels at 1 meter of orange ball
		self.lastImageTime = time.time()
		self.imageWaitTime = .01
		self.pinkhueVal = 168 #175 for hand, 168 for kinect, may need to continually tune
		self.bluehueVal = 110
		#160 #pink

		#topic for the raw image/camera/depth_registered/image_raw
		#try camera/rgb/image_color/compressed for greater efficiency
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color",Image,self.imagecallback, queue_size=1)
		self.depth_image_sub = rospy.Subscriber("/camera/depth_registered/sw_registered/image_rect", Image, self.depthcallback, queue_size=10)
		self.depth_image = None
		#self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image",Image,self.imagecallback, queue_size=1)
		print "subscribed to /camera/rgb/image_rect_color"
		print "done initializing"

		cv2.startWindowThread()
		cv2.namedWindow('HSV_Mask_PINK_BALL')

		cv2.startWindowThread()
		cv2.namedWindow('HSV_Mask_BLUE_BLOCKS')


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
		ball_pose = self.findBall(cv_image)
		self.lastImageTime = time.time()


	def depthcallback(self,data):

		#if self.lastDepthTime > time.time() - self.depthWaitTime :
		#	return
		#print "depth callback"
		try:

			self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")#rgba8
			#print "Updated depth image"
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

		hue_range = 5
		colorLower = (hueVal-hue_range, 100, 100)
		colorUpper =(hueVal+hue_range, 255, 255)
		blurred = cv2.GaussianBlur(rgbimage, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, colorLower, colorUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

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
			cnts.remove(c)
			#except :
				#pass
			
			((x, y), radius) = cv2.minEnclosingCircle(c)
	 		lookfor = lookfor - 1
			
			if radius > 5:# only proceed if the radius meets a minimum size
				blobsFound.append([x,y,radius])
				cv2.circle(res, (int(x), int(y)), int(radius), (0,255,255), 2)

		cv2.imshow(hsvstring,res)
		return blobsFound

	#find a ball and return its transform relative to the camera
	#http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
	def findBall(self, rgbimage) :
		
		initialtime = time.time()
	
		"""blockList = self.findBlobsofHue(self.bluehueVal, 6, rgbimage)
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
		
		try:
			if block_poses_list != [] :
				block_poses = PoseArray()
				block_poses.header = Header()
				block_poses.poses = block_poses_list
				self.block_pub.publish(block_poses)
		except CvBridgeError, e:
			print e
		"""

		#used for tuning
		# pinkmax = 169
		# pinkmin = 168
		# self.pinkhueVal = (self.pinkhueVal + 1) % pinkmax
		# if self.pinkhueVal < pinkmin :
		# 	self.pinkhueVal = pinkmin

		# print "Pink Hue Val"
		# print self.pinkhueVal


		pink_balls = self.findBlobsofHue(self.pinkhueVal, 1, rgbimage)
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
					self.ball_pub.publish(ball_pose)
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
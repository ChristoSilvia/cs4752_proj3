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
from copy import deepcopy


#http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython



class kinect_calibration:

	def __init__(self):
		print "initializing kinect_calibration"
		
		self.get_calibration_points = createService("get_calibration_points", GetCalibrationPoints, self.getCalibrationPoints, "")
	
	def imagecallback(self,data):
		try:
			self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			self.draw_image = self.rgb_image.copy()
			self.lastImageTime = time.time()
		except CvBridgeError, e:
			print e

	def onmouse(self,event,x,y,flags,params):
		if event == cv2.EVENT_LBUTTONDOWN:
			self.click_down = True
			# print "Left click down"

			self.draw_image = self.rgb_image.copy()
			cv2.circle(self.draw_image, (int(x), int(y)), int(self.radius), (0,255,255),2)

		elif event == cv2.EVENT_MOUSEMOVE:
			if self.click_down:
				# print "dragging"
				world_point = self.screenToWorld(x,y)
				print world_point

				self.draw_image = self.rgb_image.copy()
				cv2.circle(self.draw_image, (int(x), int(y)), int(self.radius), (0,255,255),2)

		elif event == cv2.EVENT_LBUTTONUP:
			self.click_down = False
			# print "Final World Point"
			world_point = self.screenToWorld(x,y)
			# print world_point

			self.draw_image = self.rgb_image.copy()
			cv2.circle(self.draw_image, (int(x), int(y)), int(self.radius), (0,255,255),2)
			self.last_point = [x,y]

	def getCalibrationPoints(self, req):
		points = req.point_names
		self.bridge = CvBridge()

		cv2.startWindowThread()
		self.pointstring = 'Playing Field Calibration Points'
		cv2.namedWindow(self.pointstring)

		cv2.setMouseCallback(self.pointstring, self.onmouse)

		self.click_down = False
		self.radius = 10

		self.last_point = []

		self.calibration_points = []
		point_count = 0
		prompt = True

		self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color",Image,self.imagecallback, queue_size=1)
		self.depth_image_sub = rospy.Subscriber("/camera/depth_registered/hw_registered/image_rect", Image, self.depthcallback, queue_size=1)
		self.depth_image = None
		self.rgb_image = None

		rate = rospy.Rate(5)
		while self.depth_image == None or self.rgb_image == None :
			print "Waiting for images"
			rate.sleep()

		self.image_sub.unregister()
		self.depth_image_sub.unregister()

		while point_count < len(points):
			point_name = points[point_count]
			# display the image and wait for a keypress
			cv2.imshow(self.pointstring,self.draw_image)
			if prompt:
				print "Please click on the %s and press space" % point_name
				prompt = False

			key = cv2.waitKey(1) & 0xFF

			# # if the space key is pressed, add the point to the dict
			if key == ord(" "):
				x = self.last_point[0]
				y = self.last_point[1]

				cv2.circle(self.rgb_image, (int(x), int(y)), int(self.radius), (0,0,255),2)
				self.draw_image = self.rgb_image.copy()
				world_point = self.screenToWorld(x,y)

				self.calibration_points.append(world_point)
				point_count += 1

				print point_name
				print world_point
				prompt = True

		# destroy window
		cv2.destroyAllWindows()

		return GetCalibrationPointsResponse(self.calibration_points)

	def screenToWorld(self, x,y) :
		bi = [x,y]
		if self.depth_image != None :
			distance = self.depth_image[int(bi[1]), int(bi[0])]
			if not math.isnan(distance) :
				world_point = self.projectDepth((int(bi[0]), int(bi[1])), distance, self.rgb_image.shape[1], self.rgb_image.shape[0])
 			else :
 				"error nan"
 		else :
 			"error no depth image"

 		return world_point.position

	def depthcallback(self,data):
		try:
			self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")#rgba8
		except CvBridgeError, e:
			print e

	def projectDepth(self, point, distance, width, height) :
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

def main(args):
  
  rospy.init_node('kinect_calibration', anonymous=True)
  print "initialized node kinect_calibration"
  ic = kinect_calibration()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

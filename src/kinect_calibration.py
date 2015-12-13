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
import tf
from tf.transformations import *
from config import *
from copy import deepcopy
import baxter_interface
from baxter_interface import *

class kinect_calibration:

	def __init__(self):
		print "initializing kinect_calibration"

		self.limb_name = rospy.get_param("limb")
		self.limb_name = 'left'
		self.arm = baxter_interface.Limb(self.limb_name)
		self.transform_listener = tf.TransformListener()
		self.tf_br = tf.TransformBroadcaster()

		req = CalibrateKinect()
		req.calibration_points = ["BOTTOM_MIDDLE", "BOTTOM_CORNER", "TOP_MIDDLE", "TOP_CORNER"]
		
		calibrate_kinect = createService("calibrate_kinect", CalibrateKinect, self.calibrate, "")
		self.calibrate(req)

		self.ts = None
		self.rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			if self.ts != None:
				self.tf_br.sendTransform(t)
			self.rate.sleep()

	def imagecallback(self,data):
		try:
			self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			self.draw_image = self.rgb_image.copy()
			self.lastImageTime = time.time()
		except CvBridgeError, e:
			print e

	def getToolPos(self):
		gripper_frame = "%s_gripper_base" % self.limb_name
		toolLen = .13
		ps = PointStamped()
		ps.header.frame_id = gripper_frame
		ps.header.stamp = self.transform_listener.getLatestCommonTime(gripper_frame, 'base')
		ps.point = Point(0,0,toolLen)

		toolPos = self.transform_listener.transformPoint('base', ps)
		return vector3_to_numpy(toolPos.point)

	def transform_from_matrix(self, transform):
		t = TransformStamped()
		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "/base"
		t.child_frame_id = "/camera_rgb_optical_frame"
		T = translation_from_matrix(transform)
		t.transform.translation.x = T[0]
		t.transform.translation.y = T[1]
		t.transform.translation.z = T[2]
		q = quaternion_from_matrix(transform)
		t.transform.rotation.x = q[0]
		t.transform.rotation.y = q[1]
		t.transform.rotation.z = q[2]
		t.transform.rotation.w = q[3]

		return t

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

	def calibrate(self, req):
		calibration_points = req.calibration_points
		base_points = self.get_base_frame_points()
		# print base_points

		prompt = "Press Enter when Arm is out of the way of the kinect's view of the table"
		cmd = raw_input(prompt)

		kinect_points = self.get_kinect_frame_points()
		# print kinect_points

		kinect_transform = self.get_kinect_transform(base_points, kinect_points)
		print kinect_transform

		self.playing_field = {}
		for point_name in kinect_points:
			kinect_pt = kinect_points[point_name]
			base_pt = self.KinectToBasePoint(kinect_transform, kinect_pt[0],kinect_pt[1],kinect_pt[2])
			self.playing_field[point_name] = base_pt
		print self.playing_field

		# vec1 = point_pos[1] - point_pos[0]
		# vec2 = point_pos[2] - point_pos[0]

		# self.kinect_norm = np.cross(vec1, vec2)
		# self.set_plane_normal_srv(Vector3(self.plane_norm[0], self.plane_norm[1], self.plane_norm[2]))
		# # plane_origin = np.average(point_pos, axis=0)
		# plane_origin = point_pos[0]

		# self.plane_translation = translation_matrix(plane_origin)

		# x_plane = vec1/np.linalg.norm(vec1)
		# y_plane = np.cross(self.plane_norm, vec1)
		# y_plane = y_plane/np.linalg.norm(y_plane)
		# z_plane = self.plane_norm/np.linalg.norm(self.plane_norm)
		# #need rotation to make norm the z vector
		# self.plane_rotation = np.array([x_plane, y_plane, z_plane]).T

		# self.plane_rotation = np.append(self.plane_rotation,[[0,0,0]],axis=0)
		# self.plane_rotation = np.append(self.plane_rotation,[[0],[0],[0],[1]],axis=1)

		print "#################################"
		print "Finished Calibrating Playing Field and Kinect Frame"
		print "kinect_translation"
		print translation_from_matrix(kinect_transform)
		print "kinect_rotation"
		print quaternion_from_matrix(kinect_transform)
		print "#################################"

		self.ts = self.transform_from_matrix(kinect_transform)

		return GetCalibrationPointsResponse(self.ts)


	def get_base_frame_points(self):
		calibration_points = ["BOTTOM_MIDDLE", "BOTTOM_CORNER", "TOP_MIDDLE", "TOP_CORNER"]
		point_pos = {}
		print calibration_points
		for point_name in calibration_points:
			prompt = "Press Enter when Arm is on the %s" % point_name
			cmd = raw_input(prompt)
			point_pos[point_name] = self.getToolPos()
		return point_pos

	def get_kinect_frame_points(self):
		calibration_points = ["BOTTOM_MIDDLE", "BOTTOM_CORNER", "TOP_MIDDLE", "TOP_CORNER"]
		# calibration_points = ["BOTTOM_MIDDLE", "BOTTOM_CORNER", "BALL_START", "TOP_CORNER", "BOTTOM_B_NEAR_GOAL", "TOP_B_NEAR_GOAL"]

		kinect_points = self.get_calibration_points(calibration_points)
		print "kinect_points"
		print kinect_points

		point_pos = {}
		for i in range(0,len(calibration_points)):
			point_name = calibration_points[i]
			point_pos[point_name] = vector3_to_numpy(kinect_points[i])
		return point_pos


	def get_kinect_transform(self, base_points, kinect_points):
		shape = (1, len(base_points), 3)
		source = np.zeros(shape, np.float32)
		target = np.zeros(shape, np.float32)

		count = 0
		for point_name in base_points:
			kinect_pt = list(kinect_points[point_name])
			source[0][count] = kinect_pt
			base_pt = list(base_points[point_name])
			target[0][count] = base_pt
			count += 1
		# print source
		# print target

		retval, M, inliers = cv2.estimateAffine3D(source, target)
		M = np.append(M,[[0,0,0,1]], axis=0)

		return M

	def KinectToBasePoint(self, M, kinect_x,kinect_y,kinect_z):
		# self.transform_setup()
		# M = np.dot(self.kinect_translation, self.kinect_rotation)

		kinect_coords = np.array([kinect_x,kinect_y,kinect_z,1])
		base_coords = np.dot(M, kinect_coords.T)
		base_coords = base_coords[:3]/base_coords[3]
		base_coords.reshape((1, 3))
		# print "base_coords: {0}".format(base_coords)
		return base_coords

	def get_calibration_points(self, points):
		# points = req.point_names
		self.bridge = CvBridge()

		cv2.startWindowThread()
		self.pointstring = 'Playing Field Calibration Points'
		cv2.namedWindow(self.pointstring)

		cv2.setMouseCallback(self.pointstring, self.onmouse)

		self.click_down = False
		self.radius = 10

		self.last_point = []

		self.kinect_points = []
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

				self.kinect_points.append(world_point)
				point_count += 1

				print point_name
				print world_point
				prompt = True

		# destroy window
		cv2.destroyAllWindows()

		return self.kinect_points

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

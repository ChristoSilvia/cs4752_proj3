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
import tf2_ros
from tf.transformations import *
from config import *
from baxter_core_msgs.msg import * #(SolvePositionIK, SolvePositionIKRequest)
from baxter_core_msgs.srv import *
from baxter_interface import *
from baxter_pykdl import baxter_kinematics
import baxter_interface
from cs4752_proj3.srv import *


#http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython



class single_color_vision_hand:

	def __init__(self):
		print "initializing single color vision object of HAND"

		rs = baxter_interface.RobotEnable(CHECK_VERSION)
		init_state = rs.state().enabled

		self.limb_name = rospy.get_param("limb")
		self.limb = Limb(self.limb_name)

		self.cam_x_offset = 0.045                      # camera gripper offset
		self.cam_y_offset = -0.01

		self.ball_pub = rospy.Publisher("/ball_pose_hand",PoseStamped)

		
		#self.pixel_radius = 10
		self.lastImageTime = time.time()
		self.imageWaitTime = .01

		self.pinkhueVal = 175 #175 for hand, 163 for kinect, may need to continually tune
		self.bluehueVal = 110
		self.pink_ball_pubs = 0

		self.bridge = CvBridge()	


		self.tfBuffer = tf2_ros.Buffer()
		self.transform_listener = tf2_ros.TransformListener(self.tfBuffer)


		self.image_sub = rospy.Subscriber("/cameras/"+self.limb_name+"_hand_camera/image",Image,self.imagecallback, queue_size=1)
		self.traj_srv = createServiceProxy("move_end_effector_trajectory", JointAction, self.limb_name)

		self.rgb_image = None
		self.depth_image_set = False
		
		print "subscribed to /cameras/left_hand_camera/image"
		print "done initializing"

		cv2.startWindowThread()
		cv2.namedWindow('HSV_Mask_PINK_BALL')

		cv2.startWindowThread()
		cv2.namedWindow('HSV_Mask_BLUE_BLOCKS')



	#def cameraIntrinsicsCB (self, data) :
	#	self.camera_matrix = data.k

	def get_position(self):
		return np.array(self.limb.endpoint_pose()['position']) 

	def move_arm_to_target(self, target_pose, move_time, time_delta, distance_to_goal) :
		print "Moving Arm to seen Target!"

		initial_pos = self.get_position()

		if distance_to_goal > .08 :
			speed = .03
		else :
			speed = .015
		traject = JointAction()
		#traject.reference_frame = self.limb_name
		times =[]
		positions = []
		velocities = []

		target_vec = np.array([target_pose.position.x, target_pose.position.y,target_pose.position.z])
		new_distance_to_goal = np.linalg.norm(target_vec)
		print "Distance To Goal"
		print distance_to_goal
		print new_distance_to_goal
		target_vec = target_vec/distance_to_goal
		move_vel = target_vec * (speed)

		currentTime = 0.0
		new_v = Vector3(move_vel[0], move_vel[1], move_vel[2])
		while currentTime < move_time and distance_to_goal > .02:

			velocities.append(new_v)
			current_pos = initial_pos + (move_vel * currentTime)
			new_p = Vector3(current_pos[0], current_pos[1], current_pos[2])
			positions.append(new_p)
			times.append(currentTime)
			currentTime += time_delta


		print "Starting Sending Trajectory Message"
	
		self.traj_srv(times, positions, velocities)
		
		print "Finished Sending Trajectory Message"


	def imagecallback(self,data):
		#don't need to do every frame
		if self.lastImageTime > time.time() - self.imageWaitTime :
			return

		#print "got image"
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")#rgba8
		except CvBridgeError, e:
			print e


		self.rgb_image = cv_image
		self.findObjects(cv_image)
		self.lastImageTime = time.time()


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
		cy = height /2 - height/5
		fx = cx / np.tan((xFOV/2) * np.pi / 180)
		fy = cy / np.tan((yFOV/2) * np.pi / 180)
		
		toball = np.zeros(3)
		toball[0] = (point[0] - cx) / fx
		toball[1] = (point[1] - cy) / fy
		toball[2] = 1
		toball = toball / np.linalg.norm(toball) #normalize so we can then multiply by distance
		
		pixel_radius_at_meter = 8.4
	
		distance = pixel_radius_at_meter / radius
		toball = toball * distance
		pose = Pose()
		pose.position = Point(toball[0], toball[1], toball[2])
		pose.orientation = Quaternion(0,0,0,1)
		#print "FOUND Pink BALL!!!!"
		#print toball
		return pose

	def findBlobsofHue(self, hueVal, lookfor, rgbimage) :

		hue_range = 2
		colorLower = (hueVal-hue_range, 70, 70)
		colorUpper =(hueVal+hue_range, 255, 255)
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
			#cnts.remove(c)
			#except :
				#pass
			
			((x, y), radius) = cv2.minEnclosingCircle(c)
	 		lookfor = lookfor - 1
			
			if radius > 7:# only proceed if the radius meets a minimum size
				blobsFound.append([x,y,radius])
				cv2.circle(res, (int(x), int(y)), int(radius), (0,255,255), 2)

		cv2.imshow(hsvstring,res)
		return blobsFound

	def findBlocks(self, req) :
		blockList = self.findBlobsofHue(self.bluehueVal, req.num_blocks, self.rgb_image)
		block_poses_list = []
		for block in blockList :
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
		"""pinkmax = 175
		pinkmin = 155
		self.pinkhueVal = (self.pinkhueVal + .1) % pinkmax
		if self.pinkhueVal < pinkmin :
			self.pinkhueVal = pinkmin

		print "Pink Hue Val"
		print self.pinkhueVal"""
		#self.findBlocks()

		pink_balls = self.findBlobsofHue(self.pinkhueVal, 1, rgbimage)
		if pink_balls != [] :
			bi = pink_balls[0]
			ball_pose = self.project((int(bi[0]), int(bi[1])), int(bi[2]), rgbimage.shape[1], rgbimage.shape[0])
			bpp = ball_pose.position
			to_ball = np.array([bpp.x, bpp.y, bpp.z])
			to_ball_dist = np.linalg.norm(to_ball)
			print "GOING TO BALL"
			print to_ball_dist
			
			#
			try:
				if ball_pose != None :
					self.pink_ball_pubs += 1
					ball_pose_stamped = PoseStamped()
					ball_pose_stamped.header = Header()
					ball_pose_stamped.header.seq = self.pink_ball_pubs
					ball_pose_stamped.header.stamp = rospy.Time(0)
					frame_id = self.limb_name+"_hand_camera"
					ball_pose_stamped.header.frame_id = frame_id
					ball_pose_stamped.pose = ball_pose
					#self.ball_pub.publish(ball_pose_stamped)
					trans = self.tfBuffer.lookup_transform(frame_id, 'base', rospy.Time(0), rospy.Duration(1.0))
					new_point = transformPoint(ball_pose.position, trans.transform)
					ball_pose_stamped.pose.position = Vector3(new_point[0], new_point[1], new_point[2])
					ball_pose_stamped.header.frame_id = 'base'
					self.ball_pub.publish(ball_pose_stamped)
					# trans = self.tfBuffer.lookup_transform('turtle2', 'turtle1', rospy.Time.now(), rospy.Duration(1.0))
					print "current ball pose"
					print new_point

					#self.move_arm_to_target(ball_pose, 1, .05, to_ball_dist)
					

			except CvBridgeError, e:
				print e

		
		

def transformPoint(p, transform):
	T = translation_matrix(vector3_to_numpy(transform.translation))
	R = quaternion_matrix(quaternion_to_numpy(transform.rotation))
	M = np.dot(R,T)
	v = np.array([p.x,p.y,p.z,1])
	v = np.dot(M, v.T)
	v = v[:3]/v[3]
	# v.reshape((1, 3))
	# print base
	return v

def main(args):
  
	rospy.init_node('single_color_vision_hand', anonymous=True)
	print "initialized node single color vision"
	ic = single_color_vision_hand()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
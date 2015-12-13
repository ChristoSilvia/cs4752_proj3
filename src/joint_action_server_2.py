#!/usr/bin/env python

import numpy as np
import config

import rospy

import baxter_interface

import baxter_pykdl

class JointActionServer:
	def __init__(self, limb_name):
		self.limb_name = limb_name
		self.limb = baxter_interface.Limb(limb_name)
		self.limb_kin = baxter_pykdl.baxter_kinematics(limb_name)
		
		config.createService("/move_through_poses", PoseList, self.move_through_poses, self.limb_name)

	def move_through_poses(self, args)
		poses = args.poses
		n = len(poses)

		times = np.empty(n)
		positions = np.empty(n, 3)
		velocities = np.empty(n, 3)
		orientations = np.empty(n-1, 4)
		angular_velocities = np.empty(n-1, 4)
		rotation_angles = np.empty(n-1)

		for i in xrange(n):
			times[i] = poses[i].header.stamp.secs + 1e-9*poses[i].header.stamp.nsecs
			positions[i,:] = config.vector3_to_numpy(poses[i].pose.position)
			orientations[i,:] = config.quaternion_to_numpy(poses[i].pose.orientation)

			if i is not 0:
				delta_t = times[i] - times[i-1]

				velocities[i-1,:] = (positions[i,:] - positions[i-1,:])/delta_t

				relative_orientation = config.multiply_quaternion(
					orientations[i,:],
					config.invert_unit_quaternion(orientations[i-1,:]))
				relative_orientation_angle = 2.0*np.arccos(relative_orientation[3])
				rotation_angles[i] = relative_orientation_angle

				if relative_orientation_angle < 1e-6:
					angular_velocities[i-1,:] = np.zeros(3)
				else:
					relative_orientation_axis = relative_orientation[:3]/np.sin(0.5*relative_orientation_angle)
					angular_velocities[i-1,:] = (relative_orientation_angle/delta_t)*relative_orientation_axis

		total_time = times[-1] - times[0]

		t_start = rospy.get_time()
		t_now = t_start
		i_last = 0
		desired_twist = np.empty(6)
		while (t_now - t_start) < total_time:
			if (t_now - t_start) > times[i_last]:
				i_last += 1
			delta_t = t_now - times[i]
			time_interval = times[i+1] - times[i]

			desired_position = positions[i,:] + velocities[i,:]*delta_t
			desired_orientation = orientations[i,:]*np.sin((1 - (delta_t/time_interval))*rotation_angles[i])/np.sin(rotation_angles[i]) + orientations[i+1,:]*np.sin((delta_t/time_interval)*rotation_angles[i])/np.sin(rotation_angles[i])

			

			desired_twist[:3] = velocities[i,:]
			desired_twist[3:] = angular_velocities[i,:]

			jacobian = np.array(self.limb_kin.jacobian())
			jacobian_pinv = np.linalg.pinv(jacobian)
			
			desired_joint_velocities = np.dot(jacobian_pinv, desired_twist)
			self.limb.set_joint_velocities(
				config.numpy_to_joint_dict(
					self.limb_name,
					desired_joint_velocities))

			t_now = rospy.get_time()


if __name__ == '__main__':
	rospy.init_node('joint_action_server_2')
	baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()
	print("Enabled Robot")

	try:
		limb_name = rospy.get_param("limb")
	except:
		if (sys.argv[1] == 'left' or sys.argv[1] == 'right'):
			limb_name = sys.argv[1]
		else:
			limb_name = None
			print("No Limb, Node Will Exit")

	if not (limb_name is None):
		JointActionServer(limb_name)

#!/usr/bin/env python
# Team: zic; Names: Zach Vinegar, Isaac Qureshi, Chris Silvia
import rospy
from baxter_interface import CHECK_VERSION, RobotEnable, Limb

limb_name = "left"
dt = 0.01

def record():
	rospy.init_node('record')

	RobotEnable(CHECK_VERSION).enable()
	baxter_limb = Limb(limb_name)

	while True:
        initial_time = rospy.get_time()
        forces_vec3 = baxter_limb.endpoint_effort()['force']
		forces = np.array([forces_vec3.x, forces_vec3.y, forces_vec3.z])

        print "===========\nEndpoint Forces: \nF_x: {x} \nF_y: {y} \nF_z: {z}\n===========\n".format(forces_vec3)

        after_time = rospy.get_time()
		rospy.sleep(dt - (after_time - initial_time))

#!/usr/bin/env python


import rospy

import baxter_interface

class Interceptor:
	def __init__(self, limb_name):

		# positions are in global frame
		# angles are about the z axis, right-handed, and zero at the x axis.
		target_position = np.array([0.5, 0.6, -0.8])
		target_angle = np.pi/6.0

if __name__ == '__main__':
	rospy.init_node('interceptor')
	baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()

	limb_name = rospy.get_param('limb_name')
    if limb_name is None:
        if sys.argv[1] == "left" or sys.argv[1] == "right":
            limb_name = sys.argv[1]
        else:
            assert False "No limb"

	print("Initializing Ball Interception on limb {0}".format(limb_name))
    Interception(limb_name)

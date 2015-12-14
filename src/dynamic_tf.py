#!/usr/bin/env python

import rospy
import rospy
import tf

from dynamic_reconfigure.server import Server
from cs4752_proj3.cfg import kinect_tfConfig

br = tf.TransformBroadcaster()
current_config = 0

def callback(config, level):
	global current_config
	q = tf.transformations.quaternion_from_euler(config.roll, config.pitch, config.yaw)
	config.qx = q[0]
	config.qy = q[1]
	config.qz = q[2]
	config.qw = q[3]
	config.q = q
	current_config = config

	rospy.loginfo("""rosrun tf static_transform_publisher {x} {y} {z} {qx} {qy} {qz} {qw} {parent_frame} {child_frame} 100""".format(**current_config))
	
	return current_config	

def update():
	global br
	global current_config
	br.sendTransform((current_config.x, current_config.y, current_config.z),
		current_config.q,
		rospy.Time.now(),
		current_config.child_frame,
		current_config.parent_frame)

def server():
	rospy.init_node("dynamic_tf", anonymous = True)

	rospy.sleep(5)
	
	srv = Server(kinect_tfConfig, callback)

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		update()
		rate.sleep()

if __name__ == "__main__":
	try:
		server()
	except rospy.ROSInterruptException:
		pass

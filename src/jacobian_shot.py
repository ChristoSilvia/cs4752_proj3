#!/usr/bin/env python

import config
import numpy as np
import sys
import matplotlib.pyplot as plt

import rospy

import baxter_interface

import baxter_pykdl


good_initials = {'w0': 1.7134565381103517, 'w1': 1.2732040524902344, 'w2': -1.4841264105834961, 'e0': -1.2996652210510256, 'e1': 1.4465438813232423, 's0': 0.519252496105957, 's1': 0.14035924193115235}
cocked = {'w0': 0.6032379441467286, 'w1': 1.2939127931030274, 'w2': -0.9713933328186036, 'e0': -0.8709175913269044, 'e1': 0.8594127354309082, 's0': 0.3413107249145508, 's1': 0.012655341485595705}
block = {'w0': 1.2720535669006348, 'w1': 0.9970875109863282, 'w2': -1.6191167197631837, 'e0': -0.9813642079284669, 'e1': 1.374063289178467, 's0': 0.41072335548706057, 's1': -0.1100631214050293}
hockey = {'w0': 1.1489516088134766, 'w1': 0.9464661450439453, 'w2': -1.5608254498901368, 'e0': -0.8578787546447755, 'e1': 1.3146215337158205, 's0': 0.29682528211669923, 's1': -0.17717478079833984}
right_throw = {'s0': 0.24045148822631837, 's1': -0.5526165782043457, 'w0': 0.511582592175293, 'w1': 0.6902913537597657, 'w2': -2.610835297998047, 'e0': -0.3439951912902832, 'e1': 1.5324468053466798}

class ConstantVelocityShooter:
    def __init__(self, limb_name):
        self.limb_name = limb_name
        self.limb = baxter_interface.Limb(limb_name)
        self.limb_kin = baxter_pykdl.baxter_kinematics(limb_name)

        self.gripper = baxter_interface.Gripper(limb_name)

        target_joint_pos = {}
        for key in right_throw:
            target_joint_pos[limb_name+"_"+key] = right_throw[key]

        self.limb.move_to_joint_positions(target_joint_pos)
            

        self.release_orientation = np.array([])

        self.ball_start = config.vector3_to_numpy(self.limb.endpoint_pose()['position'])
        self.desired_orientation = config.quaternion_to_numpy(self.limb.endpoint_pose()['orientation'])

        self.current_position = self.ball_start

        ts = [] 
        xs = []
        zs = []
        wzs = []

        self.field_length = 0.2

        self.y_velocity = 1.0

        # set each of these numbers to 0 and then ramp up until stable
        self.kp = np.array([ 10.0, 10.0, 25.0])

        while np.abs(self.ball_start[1] - self.current_position[1]) < self.field_length and not rospy.is_shutdown():
            ts.append(rospy.get_time())
            pose = self.limb.endpoint_pose() 
            self.current_position = config.vector3_to_numpy(pose['position'])
            self.current_orientation = config.quaternion_to_numpy(pose['orientation'])

            self.error = np.empty(3)
            # linear error
            self.error[[0,1]] = self.current_position[[0,2]] - self.ball_start[[0,2]]
            # angular error
            if np.dot(self.desired_orientation, self.current_orientation) > 0:
                relative_orientation = config.multiply_quaternion(self.desired_orientation,
                    config.invert_unit_quaternion(self.current_orientation))
            else:
                relative_orientation = config.multiply_quaternion(self.desired_orientation,
                    -config.invert_unit_quaternion(self.current_orientation))
            print(relative_orientation)
            relative_orientation_angle = 2.0 * np.arccos(relative_orientation[3])
            if np.abs(relative_orientation_angle) < 1e-6:
                self.error[2] = 0.0
            else:
                relative_orientation_axis = relative_orientation[:3]/np.sin(
                    0.5*relative_orientation_angle) 
                orientation_error = -relative_orientation_angle * relative_orientation_axis
                self.error[2] = orientation_error[2]

            # save errors
            xs.append(self.error[0])
            zs.append(self.error[1])
            wzs.append(self.error[2])
 
            jacobian = np.array(self.limb_kin.jacobian())
            jacobian_pinv = np.linalg.pinv(jacobian)
            
            twist = np.zeros(6)
            twist[1] = self.y_velocity
            twist[[0,2,5]] = -self.error * self.kp

            joint_velocities = np.dot(jacobian_pinv, twist)

            self.limb.set_joint_velocities(
                config.numpy_to_joint_dict(self.limb_name, joint_velocities))

            # release here
       
        plt.plot(ts, xs, color="red")
        plt.plot(ts, zs, color="green")
        plt.plot(ts, wzs, color="blue")
        plt.show()



if __name__ == '__main__':
    rospy.init_node('constant_velocity_shot')
    baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()
    
    limb_name = sys.argv[1]
    assert (limb_name == "left" or limb_name == "right")
    print("Initializing constant shot on limb {0}".format(limb_name))
    ConstantVelocityShooter(limb_name)

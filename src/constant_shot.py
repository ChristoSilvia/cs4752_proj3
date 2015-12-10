#!/usr/bin/env python

import config
import numpy as np
import sys

import rospy

import baxter_interface

import baxter_pykdl


good_initials = {'left_w0': 1.7134565381103517, 'left_w1': 1.2732040524902344, 'left_w2': -1.4841264105834961, 'left_e0': -1.2996652210510256, 'left_e1': 1.4465438813232423, 'left_s0': 0.519252496105957, 'left_s1': 0.14035924193115235}
cocked = {'left_w0': 0.6032379441467286, 'left_w1': 1.2939127931030274, 'left_w2': -0.9713933328186036, 'left_e0': -0.8709175913269044, 'left_e1': 0.8594127354309082, 'left_s0': 0.3413107249145508, 'left_s1': 0.012655341485595705}
block = {'left_w0': 1.2720535669006348, 'left_w1': 0.9970875109863282, 'left_w2': -1.6191167197631837, 'left_e0': -0.9813642079284669, 'left_e1': 1.374063289178467, 'left_s0': 0.41072335548706057, 'left_s1': -0.1100631214050293}
hockey = {'left_w0': 1.1489516088134766, 'left_w1': 0.9464661450439453, 'left_w2': -1.5608254498901368, 'left_e0': -0.8578787546447755, 'left_e1': 1.3146215337158205, 'left_s0': 0.29682528211669923, 'left_s1': -0.17717478079833984}

class ConstantVelocityShooter:
    def __init__(self, limb_name):
        self.limb_name = limb_name
        self.limb = baxter_interface.Limb(limb_name)
        self.limb_kin = baxter_pykdl.baxter_kinematics(limb_name)

        self.gripper = baxter_interface.Gripper(limb_name)

        self.limb.move_to_joint_positions(hockey)

        self.ball_start = config.vector3_to_numpy(self.limb.endpoint_pose()['position'])
        self.desired_orientation = np.array([-1.0/np.sqrt(2), 1.0/np.sqrt(2), 0, 0])

        self.current_position = self.ball_start

        
        self.field_length = 0.3

        self.y_velocity = 0 #-1.5

        self.kp = np.array([ 20.0, 20.0, 5.0])

        while self.current_position[1] > self.ball_start[1] - self.field_length:
           
            pose = self.limb.endpoint_pose() 
            self.current_position = config.vector3_to_numpy(pose['position'])
            self.current_orientation = config.quaternion_to_numpy(pose['orientation'])

            self.error = np.empty(3)
            # linear error
            self.error[[0,1]] = self.current_position[[0,2]] - self.ball_start[[0,2]]
            # angular error
            relative_orientation = config.multiply_quaternion(self.desired_orientation,
                config.invert_unit_quaternion(self.current_orientation))
            relative_orientation_angle = 2.0 * np.arccos(relative_orientation[3])
            if np.abs(relative_orientation_angle) < 1e-6:
                self.error[2] = 0.0
            else:
                relative_orientation_axis = relative_orientation[:3]/np.sin(
                    0.5*relative_orientation_angle) 
                orientation_error = -relative_orientation_angle * relative_orientation_axis
                self.error[2] = orientation_error[2]

 
            jacobian = np.array(self.limb_kin.jacobian())
            jacobian_pinv = np.linalg.pinv(jacobian)
            
            twist = np.zeros(6)
            twist[1] = self.y_velocity
            twist[[0,2,5]] = -self.error * self.kp

            joint_velocities = np.dot(jacobian_pinv, twist)

            self.limb.set_joint_velocities(
                config.numpy_to_joint_dict(self.limb_name, joint_velocities))



if __name__ == '__main__':
    rospy.init_node('constant_velocity_shot')
    baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()
    
    limb_name = sys.argv[1]
    assert (limb_name == "left" or limb_name == "right")
    print("Initializing constant shot on limb {0}".format(limb_name))
    ConstantVelocityShooter(limb_name)

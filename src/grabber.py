#!/usr/bin/env python


# python
import numpy as np
import matplotlib.pyplot as plt
import config

# ROS
import rospy

# BAXTER
import baxter_interface

# BAXTER kinematics
from baxter_pykdl import baxter_kinematics

# goal
start_position = {'right_s0': 0.5740923092102052, 'right_s1': -0.7876991336791993, 'right_w0': -0.028762139739990235, 'right_w1': 0.8586457450378419, 'right_w2': -3.0273110814331057, 'right_e0': 0.014956312664794923, 'right_e1': 1.4434759197509766}
class Grabber():
    def __init__(self, limb_name):
        rospy.init_node('grabber')
        baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()

        rospy.Subscriber('/ZACH_TOPIC', Vector3, self.set_target)

        self.limb_name = limb_name        
        self.limb = baxter_interface.Limb(limb_name)
        self.joint_names = self.limb.joint_names()
        self.limb_kin = baxter_kinematics(limb_name)

        self.limb.move_to_joint_positions(start_position)
        endpoint_pose = self.limb.endpoint_pose()
        self.start_location = config.vector3_to_numpy(endpoint_pose['position'])
        self.start_orientation = config.quaternion_to_numpy(endpoint_pose['orientation'])

        self.desired_position = self.start_location + np.array([0.2*np.random.rand() - 0.1, 0.2*np.random.rand() - 0.1, -0.1])
        theta = 0.0
        k = np.array([np.cos(0.5*theta), np.sin(0.5*theta), 0.0])
        self.desired_orientation = np.array([k[0], k[1], k[2], 0.0])
#        self.desired_orientation = config.quaternion_to_numpy(endpoint_pose['orientation'])

        print(self.desired_position - self.start_location)
        print(config.multiply_quaternion(self.desired_orientation,
           config.invert_unit_quaternion(self.start_orientation)))

        # x oscillates at 5.3
#        self.kp = np.array([1.5, 1.5, 3.0, 2.3, 2.3, 20.0])
        self.kp = np.array([1.5, 1.5, 3.0, 0.0, 0.0, 0.0])
        self.ki = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.kd = np.array([0.3, 0.3, 2.0, 0.0, 0.0, 0.0])
        
        ts = []
        xs = []
        ys = []
        zs = []
        wxs = []
        wys = []
        wzs = []
        qis = []
        qjs = []
        qks = []
        q1s = []

        integral = np.zeros(6)

        error = np.empty(6)

        start_time = rospy.get_time()
        last_time = start_time
        last_error = np.empty(6)
        endpoint_pose = self.limb.endpoint_pose()
        last_error[:3] = config.vector3_to_numpy(endpoint_pose['position']) - self.desired_position
        orientation = config.quaternion_to_numpy(endpoint_pose['orientation'])
        relative_orientation = config.multiply_quaternion(orientation, 
            config.invert_unit_quaternion(self.desired_orientation))
        relative_orientation_angle = 2.0*np.arccos(relative_orientation[3])
        print(relative_orientation_angle)
        relative_orientation_axis = relative_orientation[:3]/np.sin(0.5*relative_orientation_angle)
        print(relative_orientation_axis)
        orientation_error = relative_orientation_angle*relative_orientation_axis
        last_error[3:] = orientation_error

        while (rospy.get_time() - start_time) < 3.0:
            jacobian = np.array(self.limb_kin.jacobian())
            jacobian_pinv = np.linalg.pinv(jacobian)

            endpoint_pose = self.limb.endpoint_pose()
            position = config.vector3_to_numpy(endpoint_pose['position'])
            position_error = position - self.desired_position
            error[:3] = position_error
            
            orientation = config.quaternion_to_numpy(endpoint_pose['orientation'])
            relative_orientation = config.multiply_quaternion(orientation, 
                config.invert_unit_quaternion(self.desired_orientation))
            relative_orientation_angle = 2.0*np.arccos(relative_orientation[3])
            if relative_orientation_angle < 1e-6:
                 error[3:] = np.zeros(3)
            else:
                relative_orientation_axis = relative_orientation[:3]/np.sin(0.5*relative_orientation_angle)
                orientation_error = relative_orientation_angle*relative_orientation_axis
                error[3:] = orientation_error
          

            current_time = rospy.get_time()
            time_interval = current_time - last_time
            integral += error * time_interval
            last_time = current_time

            derivative = (error - last_error)/time_interval
            last_error = error

            ts.append(current_time - start_time)
            xs.append(error[0])
            ys.append(error[1])
            zs.append(error[2])
            wxs.append(error[3])
            wys.append(error[4])
            wzs.append(error[5])
            qis.append(orientation[0])
            qjs.append(orientation[1])
            qks.append(orientation[2])
            q1s.append(orientation[3])

            desired_twist = - self.kp * error - self.ki * integral - self.kd * derivative
            
            joint_velocities = np.dot(jacobian_pinv, desired_twist)

            self.limb.set_joint_velocities(
                config.numpy_to_joint_dict(self.limb_name, joint_velocities))

        plt.figure()
        plt.plot(ts, xs, color="red")
        plt.plot(ts, ys, color="green")
        plt.plot(ts, zs, color="blue")
        plt.grid(True)

        plt.figure()
        plt.plot(ts, wxs, color="orange")
        plt.plot(ts, wys, color="yellow")
        plt.plot(ts, wzs, color="pink")
        plt.grid(True)

#        plt.figure()
#        plt.plot(ts, np.ones(len(ts))*self.desired_orientation[0], linestyle='dotted', color="red")
#        plt.plot(ts, np.ones(len(ts))*self.desired_orientation[1], linestyle='dotted', color="green")
#        plt.plot(ts, np.ones(len(ts))*self.desired_orientation[2], linestyle='dotted', color="blue")
#        plt.plot(ts, np.ones(len(ts))*self.desired_orientation[3], linestyle='dotted', color="black")
#        # actual
#        plt.plot(ts, qis, color="red")
#        plt.plot(ts, qjs, color="green")
#        plt.plot(ts, qks, color="blue")
#        plt.plot(ts, q1s, color="black")
#        plt.grid(True)
        plt.show()

    def set_target(self, message):
        self.desired_position = config.vector3_to_numpy(message)
            

      
          

if __name__ == '__main__':
    Grabber('left') 

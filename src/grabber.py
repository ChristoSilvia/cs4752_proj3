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
class Blocker():
    def __init__(self, limb_name):
        rospy.init_node('blocker')
        baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()

        self.limb_name = limb_name        
        self.limb = baxter_interface.Limb(limb_name)
        self.joint_names = self.limb.joint_names()
        self.limb_kin = baxter_kinematics(limb_name)

        self.limb.move_to_joint_positions(start_position)
        self.start_location = config.vector3_to_numpy(self.limb.endpoint_pose()['position'])

        self.desired_position = self.start_location + np.array([0.2*np.random.rand() - 0.1, 0.2*np.random.rand() - 0.1, -0.1])

        print(self.desired_position - self.start_location)

        # x oscillates at 5.3
        self.kp = np.array([1.5, 1.5, 3.0])
        self.ki = np.array([0.0, 0.0, 0.0])
        self.kd = np.array([0.3, 0.3, 1.5])

        ts = []
        xs = []
        ys = []
        zs = []

        integral = np.zeros(3)

        start_time = rospy.get_time()
        last_time = start_time
        last_error = config.vector3_to_numpy(self.limb.endpoint_pose()['position']) - self.desired_position
        while (rospy.get_time() - start_time) < 3.0:
            jacobian = np.array(self.limb_kin.jacobian())
            jacobian_pinv = np.linalg.pinv(jacobian)

            position = config.vector3_to_numpy(self.limb.endpoint_pose()['position'])
            error = position - self.desired_position

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

            desired_twist = np.empty(6)
            desired_twist[0:3] = - self.kp * error - self.ki * integral - self.kd * derivative
            desired_twist[3:6] = np.zeros(3)

            joint_velocities = np.dot(jacobian_pinv, desired_twist)

            self.limb.set_joint_velocities(
                config.numpy_to_joint_dict(self.limb_name, joint_velocities))

        plt.plot(ts, xs, color="red")
        plt.plot(ts, ys, color="green")
        plt.plot(ts, zs, color="blue")
        plt.grid(True)
        plt.show()

    def set_target(self, message):
        self.desired_position[0] = self.base_frame[0] + message.x
            

      
          

if __name__ == '__main__':
    Blocker('right') 

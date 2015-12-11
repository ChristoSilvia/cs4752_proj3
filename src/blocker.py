#!/usr/bin/env python


# python
import numpy as np
import matplotlib.pyplot as plt
import config
import sys

# ROS
import rospy

# BAXTER
import baxter_interface

# BAXTER kinematics
from baxter_pykdl import baxter_kinematics

# us
from cs4752_proj3.msg import BallPositionVelocity

# goal
goal_position = { 'left_w0': -0.08705,
                  'left_w1':  0.64427,
                  'left_w2': -0.84139,
                  'left_e0':  0.07248,
                  'left_e1':  1.33341,
                  'left_s0': -0.12502,
                  'left_s1': -0.49011 }


class Blocker():
    def __init__(self, limb_name, center_of_goal):
        self.w = 0.6858
        self.l = 1.3843
        self.w_goal = 0.29845
        self.gripper_offset = 0.04
        self.center_of_goal = center_of_goal

        self.y_velocity_cutoff = 5e-3

        self.limb_name = limb_name        
        self.limb = baxter_interface.Limb(limb_name)
        self.joint_names = self.limb.joint_names()
        self.limb_kin = baxter_kinematics(limb_name)

        self.limb.move_to_joint_positions(goal_position)
        self.base_frame = config.vector3_to_numpy(self.limb.endpoint_pose()['position'])

        print(self.base_frame)

        self.desired_position = config.vector3_to_numpy(self.limb.endpoint_pose()['position'])

        rospy.Subscriber('/ball_position_velocity', BallPositionVelocity, self.set_target)

        # x oscillates at 5.3
        self.kp = np.array([2.65, 1.3, 0.9])
        self.ki = np.array([0.0, 1.5, 0.4])
        self.kd = np.array([4.2, 0.0, 0.0])

        ts = []
        xs = []
        ys = []
        zs = []

        integral = np.zeros(3)

        start_time = rospy.get_time()
        last_time = start_time
        last_error = config.vector3_to_numpy(self.limb.endpoint_pose()['position']) - self.desired_position
        while not rospy.is_shutdown()  :
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
        print("Recieved New Target: {0}".format(message.x))
        self.desired_position[0] = self.base_frame[0] + message.x
            

    def handle_position_velocity_info(self, data):
        print("\nRecieved data\n------------\n")
        if self.limb_name == "left"
            x = -(data.position.x - self.center_of_goal[0])
            y = -(data.position.y - self.center_of_goal[1])
        else:
            x = data.position.x - self.center_of_goal[0])
            y = data.position.y - self.center_of_goal[1])
            

        print("Ball Position: ({0},{1})".format(x,y))
        if np.abs(data.velocity.y) > self.y_velocity_cutoff:
            if self.limb_name == "left"
                tan_theta = (-data.velocity.x) / (data.velocity.y)
            else:
                tan_theta = ( data.velocity.x) / (-data.velocity.y)
            print("Ball Angle: {0}".format(tan_theta))
            x_blocker = self.target_position(x, y, tan_theta)
        else:
            print("Ball is Stationary")
            x_blocker = 0.0
            print("")

        if self.limb_name == "left":
            self.desired_position[0] -= x_blocker
        else:
            self.desired_position[0] += x_blocker
        print("Blocker x target: {0}".format(x_blocker))

    def impact_location_on_far_wall(self, x, y, tan_theta):
        unwalled_impact_location = x + y * tan_theta
        print("Unwalled impact location: {0}".format(unwalled_impact_location))
		
        impact_location_sign = np.sign(unwalled_impact_location)
		
        limited_impact_location = np.abs(unwalled_impact_location) % 2*self.w

        if limited_impact_location < 0.5*self.w:
            return impact_location_sign * limited_impact_location
        elif limited_impact_location < 1.5*self.w:
            return impact_location_sign * ( self.w - limited_impact_location)
        else:
            return impact_location_sign * ( limited_impact_location - 2.0*self.w)

    def target_position(self, x, y, tan_theta):
        impact_location = self.impact_location_on_far_wall(x, y, tan_theta)
        return np.clip(-0.5*self.w_goal + self.gripper_offset, 
                        0.5*self.w_goal - self.gripper_offset, 
                        impact_location)
      
          

if __name__ == '__main__':
    rospy.init_node('blocker')
    baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()
    print("Initialized node 'blocker'")

    arm_name = sys.argv[1]
    assert (arm_name == "left" or arm_name == "right")
    print("Initializing Blocker for {0} arm".format(arm_name))
    Blocker(arm_name) 

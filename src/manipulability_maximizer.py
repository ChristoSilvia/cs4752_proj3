#!/usr/bin/env python
# Team: zic; Names: Zach Vinegar, Isaac Qureshi, Chris Silvia
import numpy as np
from scipy.interpolate import PiecewisePolynomial

import matplotlib.pyplot as plt

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_core_msgs.msg import * #(SolvePositionIK, SolvePositionIKRequest)
from baxter_core_msgs.srv import *
from baxter_interface import *
from baxter_pykdl import baxter_kinematics
from cs4752_proj2.srv import *
from config import *
from tf.transformations import *
from copy import deepcopy
from datetime import datetime

class ManipulabilityMaximizer():
    def __init__(self, limb_name):
        rospy.init_node('manipulability_maximizer')
        baxter_interface.RobotEnable(CHECK_VERSION).enable()

        # limb and joint parameters
        self.limb_name = limb_name
        self.limb = baxter_interface.Limb(limb_name)
        self.limb_kin = baxter_kinematics(limb_name)
        self.joint_names = self.limb.joint_names()
        
        # time discretization paramters
        self.dt = 0.008
        self.deriv_step = 1e-5
        self.tol = 0.0
        
        self.possible_improvement = float('inf')

        self.maximize_manipulability()

    def get_gradient(self, dJdqi):
        dJsquarified = np.dot(self.jacobian, dJdqi.T) + np.dot(dJdqi, self.jacobian.T)

        trace = 0.0
        for j in xrange(6):
            trace += np.dot( self.inv_squarified_jacobian[j,:], dJsquarified[:,j]) 

        return 0.5*self.manipulability*trace 

    def get_yspeed_gradient(self, dJdqi):
        return np.sum(dJdqi[1,:])

    def maximize_manipulability(self):
        while self.possible_improvement > self.tol:
            # get jacobian
            self.jacobian = np.array(self.limb_kin.jacobian())
            # get squarified jacobian
            self.squarified_jacobian = np.dot(self.jacobian, self.jacobian.T)
            # get squarified jacobian's inverse
            self.inv_squarified_jacobian = np.linalg.inv(self.squarified_jacobian)

            # get manipulablility
            self.manipulability = np.sqrt(np.linalg.det(self.squarified_jacobian))

            self.gradient = np.empty(7)
            for i in xrange(7):
                # calculate partial deriviatve of jacobian
                angles = self.limb.joint_angles()
                angles[self.joint_names[i]] += self.deriv_step
                dJdqi = (np.array(self.limb_kin.jacobian(joint_values=angles)) - self.jacobian)/self.deriv_step

                self.gradient[i] = self.get_yspeed_gradient(dJdqi)
                

            print("Mu: {0} \tImprovement: {1}".format(self.manipulability, self.possible_improvement))

            self.null_vectors = null(self.jacobian[0:3,:])[1]

            velocity_to_match = self.gradient
            velocity_coefficients = np.empty(self.null_vectors.shape[1])

            for i in xrange(self.null_vectors.shape[1]):
                velocity_coefficients[i] = np.dot(self.null_vectors[:,i], velocity_to_match)/np.dot(velocity_to_match, velocity_to_match)
                velocity_to_match = velocity_to_match - velocity_coefficients[i]*self.null_vectors[:,i]

            joint_velocities_naked = np.dot(self.null_vectors, velocity_coefficients)
            norm_of_velocities = np.linalg.norm(joint_velocities_naked)
            if norm_of_velocities > 0.2:
                print("Norm of Velocities: 0.2")
                joint_velocities = 0.2*joint_velocities_naked/norm_of_velocities
            else:
                print("Norm of Velocities: {0}".format(norm_of_velocities))
                joint_velocities = joint_velocities_naked

            self.possible_improvement = norm_of_velocities
 
            self.limb.set_joint_velocities(
                self.make_joint_dict(joint_velocities))

    def make_joint_dict(self, joint_vector):
        joint_dict = {}
        for joint_attribute, joint_name in zip(joint_vector, self.joint_names):
            joint_dict[joint_name] = joint_attribute
        return joint_dict
    
def loginfo(message):
    rospy.loginfo(message)

def null(a, rtol=1e-5):
    # http://stackoverflow.com/questions/19820921/a-simple-matlab-like-way-of-finding-the-null-space-of-a-small-matrix-in-numpy
    u, s, v = np.linalg.svd(a)
    rank = (s > rtol*s[0]).sum()
    return rank, v[rank:].T.copy()

if __name__ == '__main__':
    try: 
        ManipulabilityMaximizer('right')
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math

from geometry_msgs.msg import PoseStamped
from ur_solve import dynamic_fun, controller


class UR:
    def __init__(self):
        pi = math.pi

        # basic--info
        num_of_dof = 3
        mass = np.zeros([num_of_dof, 1], dtype=np.float)
        mass[0, 0] = 7.1
        mass[0, 1] = 12.7
        mass[0, 2] = 4.27

        # dynamic--info
        # initial variable, t = 0
        angular = np.array([[pi/2], [0], [0]], dtype=np.float)
        angular_vel = np.zeros([3, 1], dtype=np.float)
        torque = np.zeros([3, 1], dtype=np.float)
        target_pose = PoseStamped()
        alpha = np.array([[3.27], [4.84], [0.335]], dtype=np.float)
        sk = np.array([[-0.0655], [-0.09713], [-0.01057]], dtype=np.float)

        self.angular = angular
        self.angular_velocity = angular_vel
        self.torque = torque
        self.x = target_pose.pose.position.x = 0
        self.y = target_pose.pose.position.y = -1.1843
        self.z = target_pose.pose.position.z = 0.118
        self.alpha = alpha
        self.sk = sk

    @staticmethod
    def ur_dynamic_function(angular, angular_vel, torque):
        # give the initial angular etc
        angular_new, angular_vel_new = dynamic_fun(angular, angular_vel, torque)
        return angular_new, angular_vel_new













def controller(angular, angular_velocity, step):
    q1 = angular[0, 0]
    q2 = angular[1, 0]
    q3 = angular[2, 0]
    dq1 = angular_velocity[0, 0]




ur_value = UR()
print(ur_value.angular)
print(ur_value.x)
print(ur_value.y)
print(ur_value.z)


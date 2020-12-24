#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math

from geometry_msgs.msg import PoseStamped


class UR:
    def __init__(self):
        pi = math.pi
        angular = np.array([[pi/2], [0], [0]], dtype=np.float)
        angular_velocity = np.zeros([3, 1], dtype=np.float)
        torque = np.zeros([3, 1], dtype=np.float)
        target_pose = PoseStamped()
        alpha = np.array([[3.27], [4.84], [0.335]], dtype=np.float)
        sk = np.array([[-0.0655], [-0.09713], [-0.01057]], dtype=np.float)

        self.angular = angular
        self.angular_velocity = angular_velocity
        self.torque = torque
        self.x = target_pose.pose.position.x = 0
        self.y = target_pose.pose.position.y = -1.1843
        self.z = target_pose.pose.position.z = 0.118
        self.alpha = alpha
        self.sk = sk








ur_value = UR()
print(ur_value.angular)
print(ur_value.x)
print(ur_value.y)
print(ur_value.z)
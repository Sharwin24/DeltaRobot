# Delta Robot Kinematics

import modern_robotics as mr
import numpy as np


class Vector3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class DeltaKinematics:
    def __init__(
            self,
            link_length: float,
            passive_link_length: float,
            passive_link_width: float
    ):
        self.ell = link_length
        self.L = passive_link_length
        self.W = passive_link_width


# TODO:: Define these values from real robot
P = Vector3D(0, 0, 0)  # Platform Origin
M1 = Vector3D(0, 0, 0)  # Motor 1 Origin
M2 = Vector3D(0, 0, 0)  # Motor 2 Origin
M3 = Vector3D(0, 0, 0)  # Motor 3 Origin

T_PM1 = np.array([
    [1, 0, 0, M1.x],
    [0, 1, 0, M1.y],
    [0, 0, 1, M1.z],
    [0, 0, 0, 1]
])

T_PM2 = np.array([
    [np.cos(np.pi/3), -np.sin(np.pi/3), 0, M2.x],
    [np.sin(np.pi/3), np.cos(np.pi/3), 0, M2.y],
    [0, 0, 1, M2.z],
    [0, 0, 0, 1]
])

T_PM3 = np.array([
    [np.cos(-np.pi/3), -np.sin(-np.pi/3), 0, M3.x],
    [np.sin(-np.pi/3), np.cos(-np.pi/3), 0, M3.y],
    [0, 0, 1, M3.z],
    [0, 0, 0, 1]
])


# Inverse Kinematics
# We can define two circles originating at the motor origin and the
# origin of joint 1, the intersection of two circles can be found as:
# (y1 - y0)^2 + (z1 -z0)^2 = r^2

# Where y0 refers to the motor origin and y1 refers to the joint 1 origin

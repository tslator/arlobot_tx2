#!/usr/bin/env python
"""
---------------------------------------------------------------------------------------------------
File: transforms.py

Description: Provides various common transformations
 
Author: Tim Slator
Date: 11APR17
License: MIT
---------------------------------------------------------------------------------------------------
"""
from __future__ import print_function

import math

"""
---------------------------------------------------------------------------------------------------
Imports
---------------------------------------------------------------------------------------------------
"""

# Standard
from math import pi
import tf


def normalize_angle(angle):
    """
    Normalizes specified angle (in radians) to 0 to 2*PI
    :param angle: specified angle (in radians) 
    :return: normalized angle
    """
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res <= -pi:
        res += 2.0 * pi

    return res


def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


def rpm_to_rps(rpm, radius):
    return rpm * 2 * radius * math.pi / 60


def calc_vel_inc(target, last, accel_lim, decel_lim, period):
    inc = target - last
    limit = accel_lim if inc * target > 0.0 else decel_lim
    max_inc = limit * period

    return inc, max_inc


def constrain(x, x_min, x_max):
    return min(x, x_max) if x >= 0.0 else max(x, x_min)


def normalize_vector(x, y):
    M = math.sqrt(x ** 2 + y ** 2)
    try:
        _x = math.fabs(x) / M
        _y = math.fabs(y) / M
    except ZeroDivisionError:
        return x, y

    return _x, _y

def quat_to_angle(quat):
    _, _, yaw = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw


def module_test():
    print(__name__, "Module Test")


# --- EOF ---

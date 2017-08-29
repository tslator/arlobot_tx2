#!/usr/bin/env python
"""
---------------------------------------------------------------------------------------------------
 File: odompub.py

Description: Encapsulates odometry publication into a helper calls

Author: Tim Slator
Date: 11APR17
License: MIT
---------------------------------------------------------------------------------------------------
"""
from __future__ import print_function

"""
---------------------------------------------------------------------------------------------------
Imports
---------------------------------------------------------------------------------------------------
"""

# Standard
import math

# Third-Party
from rospy import Publisher, Time, loginfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose, Point, Twist, Vector3
from tf import TransformBroadcaster, transformations


"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]
ODOM_POSE_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0, 
                         0, 1e-3, 1e-9, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e-9]

ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0, 
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e3]
ODOM_TWIST_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0, 
                          0, 1e-3, 1e-9, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9]

class OdometryPublisher:
    def __init__(self, queue_size=5):
        self._publisher = Publisher("odom", Odometry, queue_size=queue_size)
        self._broadcaster = TransformBroadcaster()
        self._odom_msg = Odometry()
        self._odom_msg.header.frame_id = "odom"
        self._odom_msg.child_frame_id = "base_link"

    def _msg(self, now, x_dist, y_dist, quat, v, w):
        """
        Updates the Odometry message
        :param now: Current time 
        :param x_dist: Current x distance traveled 
        :param y_dist: Current y distance traveled
        :param quat: Quaternion of the orientation
        :param v: Linear velocity
        :param w: Angular velocity
        :return: Odometry message
        """
        self._odom_msg.header.stamp = now
        self._odom_msg.pose.pose = Pose(Point(x_dist, y_dist, 0.0), Quaternion(*quat))
        self._odom_msg.twist.twist = Twist(Vector3(v, 0, 0), Vector3(0, 0, w))
   
        if x_dist == 0 and y_dist == 0 and v == 0 and w == 0:
            self._odom_msg.pose.covariance = ODOM_POSE_COVARIANCE2
            self._odom_msg.twist.covariance = ODOM_TWIST_COVARIANCE2
        else:
            self._odom_msg.pose.covariance = ODOM_POSE_COVARIANCE
            self._odom_msg.twist.covariance = ODOM_TWIST_COVARIANCE

        return self._odom_msg

    def publish(self, cdist, v, w, heading, quat, log=False):
        """
        Publishes an Odometry message
        :param cdist: Center (or average) distance of wheel base
        :param v: Linear velocity
        :param w: Angular velocity
        :param heading: Heading (or yaw)
        :param quat: Quaternion of orientation
        :return: None
        """
        now = Time.now()

        x_dist = cdist * math.cos(heading)
        y_dist = cdist * math.sin(heading)
        
        self._broadcaster.sendTransform((x_dist, y_dist, 0), 
                                         quat, 
                                         now,
                                         "base_footprint",
                                         "odom")

        self._publisher.publish(self._msg(now, x_dist, y_dist, quat, v, w))

        if log:
            loginfo("Publishing Odometry: heading {:02.3f}, dist {:02.3f}, velocity {:02.3f}/{:02.3f}".format(
                heading,
                cdist,
                v, w))


def module_test():
    print(__name__, "Module Test")


# --- EOF ---

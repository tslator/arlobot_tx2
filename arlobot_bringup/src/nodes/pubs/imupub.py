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
# None

# Third-Party
from rospy import Publisher, Time, loginfo
from std_msgs.msg import Header
from sensor_msgs.msg import Imu


class ImuPublisher(object):
    def __init__(self, topic='imu', frame_id='base_footprint'):
        self._publisher = Publisher(topic, Imu, queue_size = 1)

        self._imu_msg = Imu()
        self._imu_msg.header = Header(frame_id=frame_id, stamp=Time.now())


    def _msg(self, orient, linaccel, angvel):
        """
        Create an Imu message
        :param orient: IMU orientation (x, y, z, w)
        :param linaccel: IMU linear acceleration (x, y, z)
        :param angvel: IMU angular velocity (x, y, z)
        :return: Imu message
        """
        self._imu_msg.header.stamp = Time.now()
        self._imu_msg.orientation_covariance = (-1., )*9
        self._imu_msg.linear_acceleration_covariance = (-1., )*9
        self._imu_msg.angular_velocity_covariance = (-1., )*9

        self._imu_msg.orientation.w = orient.w
        self._imu_msg.orientation.x = orient.x
        self._imu_msg.orientation.y = orient.y
        self._imu_msg.orientation.z = orient.z

        self._imu_msg.linear_acceleration.x = linaccel.x
        self._imu_msg.linear_acceleration.y = linaccel.y
        self._imu_msg.linear_acceleration.z = linaccel.z

        self._imu_msg.angular_velocity.x = angvel.x
        self._imu_msg.angular_velocity.y = angvel.y
        self._imu_msg.angular_velocity.z = angvel.z

        return self._imu_msg

    def publish(self, orient, linaccel, angvel):
        """
        Publish an Imu message
        :param orient: IMU orientation (x, y, z, w)
        :param linaccel: IMU linear acceleration (x, y, z)
        :param angvel: IMU angular velocity (x, y, z)
        :return: None
        """
        if orient is not None and linaccel is not None and angvel is not None:
            self._publisher.publish(self._msg(orient, linaccel, angvel))


def module_test():
    print(__name__, "Module Test")

# --- EOF ---

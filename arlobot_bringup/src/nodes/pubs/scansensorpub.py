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
from math import pi

# Third-Party
from rospy import Publisher, Time, get_time, get_param, loginfo
from sensor_msgs.msg import LaserScan

"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

class ScanSensorPublisherError(Exception):
    pass


class ScanSensorPublisher(object):
    def __init__(self, 
                 name, 
                 frame_id, 
                 min_angle, 
                 max_angle, 
                 angle_increment, 
                 min_range, 
                 max_range, 
                 scan_time=None, 
                 time_increment=None, 
                 queue_size=10):

        # Create the publisher
        self._publisher = Publisher(name, LaserScan, queue_size=queue_size)

        # Create a the laser scan message.  Most of the message is static, the dynamic
        # fields are updated in _msg().
        self._scan_msg = LaserScan()
        self._scan_msg.header.frame_id = frame_id
        self._scan_msg.angle_max = max_angle
        self._scan_msg.angle_min = min_angle
        self._scan_msg.angle_increment = angle_increment
        self._scan_msg.range_min = float(min_range)
        self._scan_msg.range_max = float(max_range)
        # Note: scan_time is the time between scans of the laser, i.e., the time it takes to read 360 degrees.
        self._scan_msg.scan_time = scan_time
        # Note: time_increment is the time between measurements, i.e. how often we read the scan and publish it (in
        # seconds)
        self._scan_msg.time_increment = time_increment
        # Per ROS documentation, the intensities filed should remain empty unless utilized by the sensor
        self._scan_msg.intensities = []

        # Storage of the last time for calculating the scan time
        self._last_scan_time = Time.now()

    def _msg(self, ranges, intensities):
        now = Time.now()
        self._scan_msg.header.stamp = now 

        # Calculate the scan time and set it in the message
        scan_time = now - self._last_scan_time
        self._last_scan_time = now        
        self._scan_msg.scan_time = scan_time.to_sec()

        self._scan_msg.ranges = [min(max(range, self._scan_msg.range_min), self._scan_msg.range_max) for range in ranges]
        if intensities is not None:
            self._scan_msg.intensities = intensities

        return self._scan_msg

    def publish(self, values, intensities=None):
        self._publisher.publish(self._msg(values, intensities))


class LaserScanPublisher(ScanSensorPublisher):
    def __init__(self, name, queue_size=10):

        # Get default values from parameter server
        min_angle = get_param("Laser Min Angle", 0.0)
        max_angle = get_param("Laser Max Angle", 2*pi)
        angle_increment = get_param("Laser Angle Increment", 2*pi/360.0)
        min_range = get_param("Laser Min Range", 0.0)
        max_range = get_param("Laser Max Range", 10.0)
        scan_time = get_param("Laser Scan Time", 0.1)
        increment_time = get_param("Laser Increment Time", 0.001) 

        super(LaserScanPublisher, self).__init__("laser", 
                                                 name+"_laser", 
                                                 min_angle, 
                                                 max_angle, 
                                                 angle_increment, 
                                                 min_range, 
                                                 max_range, 
                                                 scan_time, 
                                                 queue_size)


class UltrasonicScanPublisher(ScanSensorPublisher):
    # Note: The values in offsets are in degrees relative to the center of the robot.
    # Each entry is the position of one or more sensors.
    _OFFSETS = [60, 30, 0, 330, 300, 45, 0, 315, 240, 210, 180, 150, 120, 225, 180, 135]

    def __init__(self, queue_size=10):

        # Get default values from parameter server
        min_angle = get_param("Ultrasonic Min Angle", 0.0)
        max_angle = get_param("Ultrasonic Max Angle", 2*pi)
        angle_increment = get_param("Ultrasonic Angle Increment", 0.0174)
        min_range = get_param("Ultrasonic Min Range", 0.1)
        max_range = get_param("Ultrasonic Max Range", 4.0)
        increment_time = get_param("Ultrasonic Increment Time", 0.001)

        super(UltrasonicScanPublisher, self).__init__("ultrasonic_scan", 
                                                       "ultrasonic_array", 
                                                       min_angle, 
                                                       max_angle, 
                                                       angle_increment, 
                                                       min_range, 
                                                       max_range, 
                                                       increment_time, 
                                                       queue_size)

    def publish(self, values):
        """
        Create an equivalent list of values as for a laser replacing values corresponding 
        to the offsets of the sensor positions
        """

        # Default all values to furthest distance and update with actual sensor values
        ranges = [self._scan_msg.range_max]*360
        for index, value in enumerate(values):
            offset = UltrasonicScanPublisher._OFFSETS[index]
            # Note: There are duplicates in the offsets, e.g., sensors at the same angle but different height.
            # We take the smallest values.
            ranges[offset] = min(value, ranges[offset])

        super(UltrasonicScanPublisher, self).publish(ranges)


class InfraredScanPublisher(ScanSensorPublisher):
    # Note: The values in offsets are in degrees relative to the center of the robot.
    # Each entry is the position of one or more sensors.
    _OFFSETS = [330, 30, 45, 15, 345, 315, 330, 30, 150, 210, 225, 195, 165, 135, 150, 210]

    def __init__(self, queue_size=10):

        # Get default values from parameter server
        min_angle = get_param("Infrared Min Angle", 0.0)
        max_angle = get_param("Infrared Max Angle", 2*pi)
        angle_increment = get_param("Infrared Angle Increment", 0.0174)
        min_range = get_param("Infrared Min Range", 0.01)
        max_range = get_param("Infrared Max Range", 0.8)
        increment_time = get_param("Infrared Increment Time", 0.001)

        super(InfraredScanPublisher, self).__init__("infrared_scan", 
                                                    "infrared_array", 
                                                    min_angle, 
                                                    max_angle, 
                                                    angle_increment, 
                                                    min_range, 
                                                    max_range, 
                                                    increment_time, 
                                                    queue_size)

    def publish(self, values):
        """
        Create an equivalent list of values as for a laser replacing values corresponding 
        to the offsets of the sensor positions
        """

        # Default all values to furthest distance and update with actual sensor values
        ranges = [self._scan_msg.range_max]*360
        for index, value in enumerate(values):
            offset = InfraredScanPublisher._OFFSETS[index]
            # Note: There are duplicates in the offsets, e.g., sensors at the same angle but different height.
            # We take the smallest values.
            ranges[offset] = min(value, ranges[offset])

        super(InfraredScanPublisher, self).publish(ranges)


def module_test():
    lsp = LaserScanPublisher('scanse')
    usp = UltrasonicScanPublisher()
    isp = InfraredScanPublisher()

# --- EOF ---
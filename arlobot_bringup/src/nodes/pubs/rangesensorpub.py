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
from rospy import loginfo, get_param, Publisher, Time
from sensor_msgs.msg import Range

"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

class RangePublisherError(Exception):
    pass


class RangePublisher(object):
    """
    Base class for publising Range messages
    """
    def __init__(self, name, fov, min_dist, max_dist, rad_type, queue_size):
        self._name = name
        self._range_msg = Range()
        self._range_msg.radiation_type = rad_type
        self._range_msg.field_of_view = fov
        self._range_msg.min_range = min_dist
        self._range_msg.max_range = max_dist

        self._publisher = Publisher(name, Range, queue_size=queue_size)

    def _msg(self, index, distance):
        msg = self._range_msg
        msg.header.frame_id = "{}_{}".format(self._name, index)
        msg.header.stamp = Time.now()
        msg.range = min(max(distance, msg.min_range), msg.max_range)

        return msg

    def publish(self, index, distance):
        self._publisher.publish(self._msg(index, distance))


class UltrasonicArrayPublisher(RangePublisher):
    """
    Ultrasonic Array Range publisher publishes an array of ultrasonic sensor ranges
    """
    def __init__(self, queue_size=16):
        fov = get_param("Ultrasonic Field of View", 0.5233)
        min_dist = get_param("Ultrasonic Min Range", 0.1)
        max_dist = get_param("Ultrasonic Max Range", 4.0)
        super(UltrasonicArrayPublisher, self).__init__("ultrasonic_range_array", 
                                                       fov, 
                                                       min_dist, 
                                                       max_dist, 
                                                       rad_type=Range.ULTRASOUND, 
                                                       queue_size=queue_size)

    def publish(self, distances):
        for index, distance in enumerate(distances):
            super(UltrasonicArrayPublisher, self).publish(index, distance)


class InfraredArrayPublisher(RangePublisher):
    """
    Infrared Array Range publisher publishes an array of infrared sensor ranges
    """
    def __init__(self, queue_size=16):
        fov = get_param("Infrared Field of View", 0.1744)
        min_dist = get_param("Infrared Min Range", 0.01)
        max_dist = get_param("Infrared Max Range", 0.80)
        super(InfraredArrayPublisher, self).__init__("infrared_range_array", 
                                                     fov, 
                                                     min_dist, 
                                                     max_dist, 
                                                     rad_type=Range.INFRARED, 
                                                     queue_size=queue_size)

    def publish(self, distances):
        for index, distance in enumerate(distances):
            super(InfraredArrayPublisher, self).publish(index, distance)


def module_test():
    usap = UltrasonicArrayPublisher()
    ifap = InfraredArrayPublisher()

# --- EOF ---

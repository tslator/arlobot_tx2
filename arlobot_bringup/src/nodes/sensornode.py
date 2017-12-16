#!/usr/bin/env python
"""
---------------------------------------------------------------------------------------------------
File: sensornode.py

Description: Provides implementation of the SensorNode.

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
import rospy

# Project
from common import BaseNode
from imustate import ImuState
from sensorstate import UltrasonicSensorState, InfraredSensorState

"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""


class SensorNodeError(Exception):
    pass


class SensorNode(BaseNode):
    """
    The Sensor node aggregates all sensor messages received from the HAL node and publishes the 
    data as ROS messages.
        - Imu data published as ImuData
        - Ultrasonic data published as Range sensor and LaserScan
        - Infrared data published as Range sensor and LaserScan
    """
    def __init__(self, debug=False):
        super(SensorNode, self).__init__(name='SensorNode', debug=debug)

        rospy.loginfo("SensorNode init")

        self._rate = rospy.Rate(10)

        # Imu State variable used to manage subscription, publication, and state
        self._imu_state = ImuState()
        # Ultrasonic/Infrared Sensor State variables used to manage subscription, publication and state
        self._ultrasonic_sensor_state = UltrasonicSensorState()
        self._infrared_sensor_state = InfraredSensorState()

    def start(self):
        """
        Performs operations that must happen before the main loop starts but after __init__
        """
        rospy.loginfo("SensorNode starting ...")
        rospy.loginfo("SensorNode started")

    def loop(self):
        """
        Runs the node main loop
        """
        rospy.loginfo("SensorNode starting loop ...")
        while not rospy.is_shutdown():

            self._imu_state.publish()
            self._ultrasonic_sensor_state.publish()
            self._infrared_sensor_state.publish()

            self._rate.sleep()

        self.shutdown()
        rospy.loginfo("SensorNode exiting loop")

    def shutdown(self):
        """
        Performs shutdown on any resources acquired
        """
        rospy.loginfo("SensorNode shutdown")


# --- EOF ---

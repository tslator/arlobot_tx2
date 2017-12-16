#!/usr/bin/env python
from __future__ import print_function
"""
---------------------------------------------------------------------------------------------------
File: halnode.py

Description: Provides implementation of the HALNode which is the common access point for all 
hardware devices.

Author: Tim Slator
Date: 11APR17
License: MIT
---------------------------------------------------------------------------------------------------
"""

"""
---------------------------------------------------------------------------------------------------
Imports
---------------------------------------------------------------------------------------------------
"""

# Standard

# Third-Party
import rospy

# Project
from arlobot_ws.src.common.basenode import BaseNode
from pubs.halpub import (
    # Psoc Publishers
    HALStatusInPublisher, HALSpeedInPublisher, HALPositionInPublisher, HALHeadingInPublisher, HALHeartbeatInPublisher,
    HALSpeedOutPublisher, HALControlOutPublisher,
    # Imu Publishers
    HALOrientationInPublisher, HALLinearAccelInPublisher, HALAngularVelocityInPublisher, HALMagneticInPublisher,
    HALEulerInPublisher, HALTempInPublisher,
    # Sensor Publishers
    HALSensorArrayInPublisher
)
import hw.messages as hwmsg
import pubs.halpub as halpub
from common import Logger

"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

class PubTestNodeError(Exception):
    pass


class PubTestNode(BaseNode):
    """
    Hardware Abstraction Layer (HAL) node.  Exposes messages for sending and receiving data from
    hardware devices.
    """
    def __init__(self, debug=False):
        super(PubTestNode, self).__init__(name='PubTestNode', debug=debug)

        rospy.loginfo("PubTestNode init")
        self._rate = rospy.Rate(1)

        self._statusin_pub = HALStatusInPublisher()
        self._speedin_pub = HALSpeedInPublisher()
        self._posin_pub = HALPositionInPublisher()
        self._headingin_pub = HALHeadingInPublisher()
        self._heartbeatin_pub = HALHeartbeatInPublisher()
        self._speedout_pub = HALSpeedOutPublisher()
        self._controlout_pub = HALControlOutPublisher()
        self._orientin_pub = HALOrientationInPublisher()
        self._accelin_pub = HALLinearAccelInPublisher()
        self._angvelin_pub = HALAngularVelocityInPublisher()
        self._magneticin_pub = HALMagneticInPublisher()
        self._eulerin_pub = HALEulerInPublisher()
        self._tempin_pub = HALTempInPublisher()
        self._sensorin_pub = HALSensorArrayInPublisher()

    def start(self):
        pass

    def loop(self):
        while not rospy.is_shutdown():

            self._statusin_pub.publish(hwmsg.StatusData(device=0x5555, calibration=0x3333))
            self._speedin_pub.publish(hwmsg.SpeedData(left=3.1415, right=3.1415))
            self._posin_pub.publish(hwmsg.PositionData(x=3.1415, y=3.1415))
            self._headingin_pub.publish(hwmsg.HeadingData(heading=3.1415))
            self._heartbeatin_pub.publish(hwmsg.HeartbeatData(heartbeat=12345))
            self._speedout_pub.publish(hwmsg.SpeedData(left=3.1415, right=3.1415))
            self._controlout_pub.publish(hwmsg.ControlData(device=0xAAAA, debug=0xCCCC))
            self._orientin_pub.publish(hwmsg.OrientationData(x=3.1415, y=3.1415, z=3.1415, w=3.1415))
            self._accelin_pub.publish(hwmsg.LinearAccelData(x=3.1415, y=3.1415, z=3.1415))
            self._angvelin_pub.publish(hwmsg.AngularVelocityData(x=3.1415, y=3.1415, z=3.1415))
            self._magneticin_pub.publish(hwmsg.MagneticData(x=3.1415, y=3.1415, z=3.1415))
            self._eulerin_pub.publish(hwmsg.EulerData(yaw=3.1415, pitch=3.1415, roll=3.1415))
            self._tempin_pub.publish(hwmsg.TemperatureData(f=3.1415, c=3.1415))
            self._sensorin_pub.publish(hwmsg.SensorData(source='infrared', data=[3.1415]*16))
            self._sensorin_pub.publish(hwmsg.SensorData(source='ultrasonic', data=[3.1415]*16))

            self._rate.sleep()

    def shutdown(self):
        pass


if __name__ == "__main__":
    try:
        node = PubTestNode(debug=True)
    except PubTestNodeError as err:
        rospy.fatal("Unable to instantiate PubTestNode - {}".format(err))
    else:
        try:
            node.start()
            node.loop()
        except rospy.ROSInterruptException as err:
            rospy.fatal("Interrupt Exception raised in PubTestNode - {}".format(err))
            node.shutdown()

# --- EOF ---

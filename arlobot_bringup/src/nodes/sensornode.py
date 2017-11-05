#!/usr/bin/env python
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
#from __future__ import print_funcion

"""
---------------------------------------------------------------------------------------------------
Imports
---------------------------------------------------------------------------------------------------
"""

# Third-Party
import rospy

# Project
from basenode import BaseNode
from arlobot_bringup.msg import (
    HALAngularVelocityIn,
    HALLinearAccelIn,
    HALOrientationIn,
    HALSensorArrayIn
)
from pubs.imupub import ImuPublisher
from pubs.rangesensorpub import UltrasonicArrayPublisher, InfraredArrayPublisher
from pubs.scansensorpub import LaserScanPublisher, UltrasonicScanPublisher, InfraredScanPublisher

"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

class ImuState(object):
    """
    Helper class to hold the 
    """
    def __str__(self):
        return "angvel: {}, linaccel: {}, orient: {}".format(self._angular_vel, self._linear_accel, self._orientation)

    def __init__(self, orient=None, angvel=None, linaccel=None):
        self._orientation = orient or HALOrientationIn()
        self._angular_vel = angvel or HALAngularVelocityIn()
        self._linear_accel = linaccel or HALLinearAccelIn()

        # Flags to ensure valid values are received for angular velocity, linear acceleration,
        # orientation before publishing Imu data
        self._angvel_ready = False
        self._linaccel_ready = False
        self._orient_ready = False

    @property
    def is_ready(self):
        return self._angvel_ready and self._linaccel_ready and self._orient_ready
    
    @property
    def orientation(self):
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        self._orientation = value
        self._orient_ready = True
    
    @property
    def angular_velocity(self):
        return self._angular_vel

    @angular_velocity.setter
    def angular_velocity(self, value):
        self._angular_vel = value
        self._angvel_ready = True

    @property
    def linear_accel(self):
        return self._linear_accel

    @linear_accel.setter
    def linear_accel(self, value):
        self._linear_accel = value
        self._linaccel_ready = True


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

        # Publishers
        # Note: The sensor node aggregates all sensors including the Imu.  Data from the Imu is used by a many different
        # nodes for different purposes.  The sensor node is publishing orientation, linear acceleration and angular
        # velocity.
        self._imu_pub = ImuPublisher(frame_id='imu_data')

        # Range Array publishers - publish each sensor individually as <sensor>_x
        self._us_pub = UltrasonicArrayPublisher()
        self._ir_pub = InfraredArrayPublisher()

        # Laser Scan publishers - publish laser scan messages substituting ultrasonic and infrared
        # range values for the laser values
        self._us_scan_pub = UltrasonicScanPublisher()
        self._ir_scan_pub = InfraredScanPublisher()
        
        self._rate = rospy.Rate(10)

        # Imu State variable used to hold accumulated imu values
        self._imu_state = ImuState()

        # Ultrasonic/Infrared sensor values for Range Array publishing
        self._ultrasonic_sensors = None
        self._infrared_sensors = None

        # Subscribers
        # Note: Subscribers come last to prent callbacks before instance attributes
        # are created.
        self._angvelin_sub = rospy.Subscriber('HALAngularVelocityIn',
                                              HALAngularVelocityIn,
                                              self._angvelin_cb)
        self._linaccelin_sub = rospy.Subscriber('HALLinearAccelIn',
                                                HALLinearAccelIn,
                                                self._linaccel_cb)
        self._orientin_sub = rospy.Subscriber('HALOrientationIn',
                                              HALOrientationIn,
                                              self._orientin_cb)

        self._sensorarrayin_sub = rospy.Subscriber('HALSensorArrayIn',
                                                   HALSensorArrayIn,
                                                   self._sensorarrayin_cb)


    def _angvelin_cb(self, msg):
        if self._imu_state:
            self._imu_state.angular_velocity = msg

    def _linaccel_cb(self, msg):
        if self._imu_state:
            self._imu_state.linear_accel = msg

    def _orientin_cb(self, msg):
        if self._imu_state:
            self._imu_state.orientation = msg

    def _sensorarrayin_cb(self, msg):
        if msg.source == 'ultrasonic':
            self._ultrasonic_sensors = msg.data
        elif msg.source == 'infrared':
            self._infrared_sensors = msg.data

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

            
            if self._imu_state.is_ready:
                self._imu_pub.publish(self._imu_state.orientation, 
                                      self._imu_state.linear_accel,
                                      self._imu_state.angular_velocity)

            if self._ultrasonic_sensors is not None:
                self._us_pub.publish(self._ultrasonic_sensors)
                self._us_scan_pub.publish(self._ultrasonic_sensors)

            if self._infrared_sensors is not None:
                self._ir_pub.publish(self._infrared_sensors)
                self._ir_scan_pub.publish(self._infrared_sensors)

            
            self._rate.sleep()

        self.shutdown()
        rospy.loginfo("SensorNode exiting loop")

    def shutdown(self):
        """
        Performs shutdown on any resources acquired
        """
        rospy.loginfo("SensorNode shutdown")


# --- EOF ---

#!/usr/bin/env python
from __future__ import print_function
"""
---------------------------------------------------------------------------------------------------
File: drivenode.py

Description: Provides implementation of the DriveNode which handles all velocity commands and 
odometry publishing.

Author: Tim Slator
Date: 04NOV17
License: MIT
---------------------------------------------------------------------------------------------------
"""

"""
---------------------------------------------------------------------------------------------------
Imports
---------------------------------------------------------------------------------------------------
"""

# Standard
from math import cos, sin

# Third-Party
import rospy

# Project
from arlobot_bringup.msg import HALSpeedIn, HALPositionIn, HALHeadingIn
from common import BaseNode
from geometry_msgs.msg import Twist

from hw.messages import SpeedData
from pubs.halpub import HALSpeedOutPublisher
from pubs.odompub import OdometryPublisher


"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""
class DriveNodeError(Exception):
    pass


class DriveNode(BaseNode):
    """
    Implements the Arlobot drive node
    """
    def __init__(self, name, debug):
        super(DriveNode, self).__init__(name=name, debug=debug)

        self.rate = rospy.get_param('loop rate', 20.0)
        self.timeout = rospy.get_param('timeout', 3.0)

        self.cmd_v = 0
        self.curr_v = 0
        self.meas_v = 0

        self.cmd_w = 0
        self.curr_w = 0
        self.meas_w = 0

        self.meas_x = 0
        self.meas_y = 0
        self.meas_th = 0

        self.now = rospy.Time.now()
        self.last_cmd = rospy.Time.now()

        self.period = 1.0/self.rate
        self._timer = rospy.Rate(self.rate)

        self._sub = rospy.Subscriber('/cmd_vel', Twist, self._cmd_vel_callback)
        self._speedin_sub = rospy.Subscriber("HALSpeedIn", HALSpeedIn, self._speedin_cb, queue_size=1)
        self._positionin_sub = rospy.Subscriber("HALPositionIn", HALPositionIn, self._positionin_cb, queue_size=1)
        self._headingin_sub = rospy.Subscriber("HALHeadingIn", HALHeadingIn, self._headingin_cb, queue_size=1)

        self._odom_pub = OdometryPublisher()
        self._speedout_pub = HALSpeedOutPublisher()

        # Create list of broadcasters and add odometry
        self._broadcasts = [
            lambda : self._odom_pub.publish(self.now, self.meas_x, self.meas_y, self.meas_v, self.meas_w, self.meas_th),
            lambda : self._speedout_pub.publish(SpeedData(linear=self.curr_v, angular=self.curr_w))
        ]

    def _cmd_vel_callback(self, msg):
        self.last_cmd = rospy.Time.now()
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z

    def _speedin_cb(self, msg):
        self.meas_v = msg.linear
        self.meas_w = msg.angular

    def _positionin_cb(self, msg):
        self.meas_x = msg.x
        self.meas_y = msg.y

    def _headingin_cb(self, msg):
        self.meas_th = msg.heading

    def _process(self):

        self.curr_v = self.cmd_v
        self.curr_w = self.cmd_w

        # Safety check if communication is lost
        if self.now > (self.last_cmd + rospy.Duration(self.timeout)):
            self._stop()

    def _stop(self):
        self.curr_v = 0
        self.curr_w = 0

    def _broadcast(self):
        for b in self._broadcasts:
            b()

    def start(self):
        self._stop()

    def loop(self):
        while not rospy.is_shutdown():
            self.now = rospy.Time.now()

            self._process()
            self._broadcast()
            self._timer.sleep()

    def shutdown(self):
        self._stop()


# --- EOF ---

#!/usr/bin/env python



from math import cos, sin, atan2, copysign, fabs

import rospy
from geometry_msgs.msg import Twist

from basenode import BaseNode
from hw.messages import SpeedData
from arlobot_bringup.msg import HALSpeedIn, HALPositionIn, HALHeadingIn
from pubs.halpub import HALSpeedOutPublisher
from pubs.odompub import OdometryPublisher
from utils.motion import velocity_smoother


class DriveNodeBase(BaseNode):
    """
    Implements the base functionality for the Arlobot drive node
    """
    def __init__(self, name, debug):
        super(DriveNodeBase, self).__init__(name=name, debug=debug)

        self.rate = rospy.get_param('loop rate', 20.0)
        self.timeout = rospy.get_param('timeout', 3.0)
        self._linear_max_accel = rospy.get_param('linear max accel', 0.5)
        self._linear_max_decel = rospy.get_param('linear max decel', 0.5)
        self._angular_max_accel = rospy.get_param('angular max accel', 0.75)
        self._angular_max_decel = rospy.get_param('linear max decel', 0.75)

        self.cmd_v = 0
        self.meas_v = 0
        self.last_v = 0
        self.cmd_w = 0
        self.meas_w = 0
        self.last_w = 0

        self.meas_x = 0
        self.meas_y = 0
        self.meas_th = 0
        self.last_th = 0

        self.now = rospy.Time.now()
        self.last_cmd = rospy.Time.now()

        self.period = 1.0/self.rate
        self._timer = rospy.Rate(self.rate)

        self._sub = rospy.Subscriber('/cmd_vel', Twist, self._cmd_vel_callback)
        self._odom_pub = OdometryPublisher()

        # Create list of broadcasters and add odometry
        self._broadcasts = [
            lambda : self._odom_pub.publish(self.now, self.meas_x, self.meas_y, self.meas_v, self.meas_w, self.meas_th)
        ]

    def _cmd_vel_callback(self, msg):
        self.last_cmd = rospy.Time.now()
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z

    def _limit_accel(self):

        v_delta, w_delta = velocity_smoother(
            self.cmd_v,
            self.last_v,
            self._linear_max_accel,
            self._linear_max_decel,
            self.cmd_w,
            self.last_w,
            self._angular_max_accel,
            self._angular_max_decel,
            self.period
        )

        self.last_v += v_delta
        self.last_w += w_delta

        self.cmd_v = self.last_v
        self.cmd_w = self.last_w

    def _process(self):
        self._limit_accel()
        if self.now > (self.last_cmd + rospy.Duration(self.timeout)):
            self._stop()

    def _stop(self):
        self.cmd_v = 0
        self.cmd_w = 0

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


class RealDriveNode(DriveNodeBase):
    """
    Implements the real drive node by communicating with the HAL node
    """
    def __init__(self, name, debug):
        super(RealDriveNode, self).__init__(name, debug)

        self._speedout_pub = HALSpeedOutPublisher()

        self._speedin_sub = rospy.Subscriber("HALSpeedIn", HALSpeedIn, self._speedin_cb, queue_size=1)
        self._positionin_sub = rospy.Subscriber("HALPositionIn", HALPositionIn, self._positionin_cb, queue_size=1)
        self._headingin_sub = rospy.Subscriber("HALHeadingIn", HALHeadingIn, self._headingin_cb, queue_size=1)

        # Add a speed out broadcast
        self._broadcasts.append(
            lambda : self._speedout_pub.publish(SpeedData(linear=self.cmd_v, angular=self.cmd_w))
        )

        rospy.loginfo("Instantiated real drive node")

    def _speedin_cb(self, msg):
        self.meas_v = msg.linear
        self.meas_w = msg.angular

    def _positionin_cb(self, msg):
        self.meas_x = msg.x
        self.meas_y = msg.y

    def _headingin_cb(self, msg):
        self.meas_th = msg.heading

    def _stop(self):
        super(RealDriveNode, self)._stop()
        self._speedout_pub.publish(SpeedData(linear=self.cmd_v, angular=self.cmd_w))


class SimulatedDriveNode(DriveNodeBase):
    """
    Implements the simulated drive node for use with Rviz and Gazebo
    """
    def __init__(self, name, debug):
        super(SimulatedDriveNode, self).__init__(name, debug)
        self.then = rospy.Time.now()

        rospy.loginfo("Instantiated simulated drive node")

    def _process(self):
        super(SimulatedDriveNode, self)._process()

        elapsed = self.now - self.then
        self.then = self.now
        elapsed = elapsed.to_sec()

        x = cos(self.meas_th) * self.cmd_v * elapsed
        y = -sin(self.meas_th) * self.cmd_v * elapsed
        self.meas_x += cos(self.meas_th) * self.cmd_v * elapsed
        self.meas_y += sin(self.meas_th) * self.cmd_v * elapsed
        self.meas_th += self.cmd_w * elapsed


# --- EOF ---
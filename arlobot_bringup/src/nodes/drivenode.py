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

        self.dx = 0
        self.last_dx = 0
        self.dr = 0
        self.last_dr = 0

        self.x = 0
        self.y = 0
        self.th = 0
        self.last_th = 0

        self.now = rospy.Time.now()
        self.last_cmd = rospy.Time.now()

        self.period = 1.0/self.rate
        self._timer = rospy.Rate(self.rate)

        self._sub = rospy.Subscriber('/cmd_vel', Twist, self._cmd_vel_callback)
        self._odom_pub = OdometryPublisher()

        # Create list of broadcasters and add odometry
        self._broadcasts = [
            lambda : self._odom_pub.publish(self.now, self.x, self.y, self.dx, self.dr, self.th)
        ]

    def _cmd_vel_callback(self, msg):
        self.last_cmd = rospy.Time.now()
        self.dx = msg.linear.x
        self.dr = msg.angular.z

    def _limit_accel(self):

        v_delta, w_delta = velocity_smoother(
            self.dx,
            self.last_dx,
            self._linear_max_accel,
            self._linear_max_decel,
            self.dr,
            self.last_dr,
            self._angular_max_accel,
            self._angular_max_decel,
            self.period
        )

        self.last_dx += v_delta
        self.last_dr += w_delta

        self.dx = self.last_dx
        self.dr = self.last_dr

    def _process(self):
        if self.now > (self.last_cmd + rospy.Duration(self.timeout)):
            self._stop()
        self._limit_accel()

    def _stop(self):
        self.dx = 0
        self.dr = 0

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
            lambda : self._speedout_pub.publish(SpeedData(linear=self.dx, angular=self.dr))
        )

        rospy.loginfo("Instantiated real drive node")

    def _speedin_cb(self, msg):
        self.dx = msg.linear
        self.dr = msg.angular

    def _positionin_cb(self, msg):
        self.x = msg.x
        self.y = msg.y

    def _headingin_cb(self, msg):
        self.th = msg.heading

    def _stop(self):
        super(RealDriveNode, self)._stop()
        self._speedout_pub.publish(SpeedData(linear=self.dx, angular=self.dr))


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

        x = cos(self.th) * self.dx * elapsed
        y = -sin(self.th) * self.dx * elapsed
        self.x += cos(self.th) * self.dx * elapsed
        self.y += sin(self.th) * self.dx * elapsed
        self.th += self.dr * elapsed


# --- EOF ---
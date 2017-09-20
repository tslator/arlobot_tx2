#!/usr/bin/env python



from math import cos, sin

import rospy
from arlobot_bringup.msg import HALSpeedIn, HALPositionIn, HALHeadingIn
from basenode import BaseNode
from geometry_msgs.msg import Twist

from motion import velocity_smoother
from hw.messages import SpeedData
from pubs.halpub import HALSpeedOutPublisher
from pubs.odompub import OdometryPublisher


class DriveNodeBase(BaseNode):
    """
    Implements the base functionality for the Arlobot drive node
    """
    def __init__(self, name, debug):
        super(DriveNodeBase, self).__init__(name=name, debug=debug)

        self.rate = rospy.get_param('loop rate', 20.0)
        self.timeout = rospy.get_param('timeout', 3.0)

        self.max_v_accel = rospy.get_param('linear max accel', 0.5) / self.rate
        self.max_v_decel = rospy.get_param('linear max decel', 0.5) / self.rate
        self.max_w_accel = rospy.get_param('angular max accel', 0.5) / self.rate
        self.max_w_decel = rospy.get_param('linear max decel', 0.5) / self.rate

        self.cmd_v = 0
        self.curr_v = 0
        self.meas_v = 0
        self.last_v = 0

        self.cmd_w = 0
        self.curr_w = 0
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

    def _process(self):

        # Limit linear/angular acceleration
        if self.curr_v < self.cmd_v:
            self.curr_v = min(self.curr_v + self.max_v_accel, self.cmd_v)
        elif self.curr_v > self.cmd_v:
            self.curr_v = min(self.curr_v - self.max_v_decel, self.cmd_v)
        else:
            self.curr_v = self.cmd_v

        if self.curr_w < self.cmd_w:
            self.curr_w = min(self.curr_w + self.max_w_accel, self.cmd_w)
        elif self.curr_w > self.cmd_v:
            self.curr_w = min(self.curr_w - self.max_w_decel, self.cmd_w)
        else:
            self.curr_v = self.cmd_v

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
            lambda : self._speedout_pub.publish(SpeedData(linear=self.curr_v, angular=self.curr_w))
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

        x = cos(self.meas_th) * self.curr_v * elapsed
        y = -sin(self.meas_th) * self.curr_v * elapsed
        self.meas_x += cos(self.meas_th) * self.curr_v * elapsed
        self.meas_y += sin(self.meas_th) * self.curr_v * elapsed
        self.meas_th += self.curr_w * elapsed


# --- EOF ---
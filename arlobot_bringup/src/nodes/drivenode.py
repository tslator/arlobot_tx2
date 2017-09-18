#!/usr/bin/env python



from math import cos, sin

import rospy
from geometry_msgs.msg import Twist

from basenode import BaseNode
from hw.messages import SpeedData
from arlobot_bringup.msg import HALSpeedIn, HALPositionIn, HALHeadingIn
from pubs.halpub import HALSpeedOutPublisher
from pubs.odompub import OdometryPublisher


class DriveNodeBase(BaseNode):
    """
    Implements the base functionality for the Arlobot drive node
    """
    def __init__(self, name, debug):
        super(DriveNodeBase, self).__init__(name=name, debug=debug)

        self.rate = 20.0
        self.timeout = 3.0

        self.x = 0
        self.y = 0
        self.th = 0
        self.dx = 0
        self.dr = 0

        self.last_cmd = rospy.Time.now()
        self._timer = rospy.Rate(self.rate)

        self._sub = rospy.Subscriber('/cmd_vel', Twist, self._cmd_vel_callback)
        self._odom_pub = OdometryPublisher()

    def _cmd_vel_callback(self, msg):
        self.last_cmd = rospy.Time.now()
        self.dx = msg.linear.x
        self.dr = msg.angular.z

    def _update(self, now):
        if now > (self.last_cmd + rospy.Duration(self.timeout)):
            self._stop()

    def _stop(self):
        self.dx = 0
        self.dr = 0

    def _broadcast(self, now):
        self._odom_pub.publish(now, self.x, self.y, self.dx, self.dr, self.th)

    def start(self):
        self._stop()

    def loop(self):
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            self._update(now)
            self._broadcast(now)
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
        self._speedout_pub.publish(Twist())

    def _cmd_vel_callback(self, msg):
        super(DriveNodeBase, self)._cmd_vel_callback(msg)
        self._speedout_pub.publish(SpeedData(linear=self.dx, angular=self.dr))


class SimulatedDriveNode(DriveNodeBase):
    """
    Implements the simulated drive node for use with Rviz and Gazebo
    """
    def __init__(self, name, debug):
        super(SimulatedDriveNode, self).__init__(name, debug)
        self.then = rospy.Time.now()

        rospy.loginfo("Instantiated simulated drive node")

    def _update(self, now):
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.to_sec()

        x = cos(self.th) * self.dx * elapsed
        y = -sin(self.th) * self.dx * elapsed
        self.x += cos(self.th) * self.dx * elapsed
        self.y += sin(self.th) * self.dx * elapsed
        self.th += self.dr * elapsed

        super(SimulatedDriveNode, self)._update(now)

# --- EOF ---
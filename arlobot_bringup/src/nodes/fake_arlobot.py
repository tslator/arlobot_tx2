#! /usr/bin/env python

from math import sin, cos

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

from basenode import BaseNode
from arlobot_bringup.msg import HALSpeedIn, HALPositionIn, HALHeadingIn
from pubs.halpub import HALSpeedOutPublisher
from hw.messages import SpeedData


class FakeArlobotNodeError(Exception):
    pass


class FakeArlobotNode(BaseNode):
    def __init__(self):
        super(FakeArlobotNode, self).__init__(name='FakeArlobotNode', debug=None)

        self.last_cmd = rospy.Time.now()
        self.rate = 20.0
        self.timeout = 3.0
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta

        self._sub = rospy.Subscriber('/cmd_vel', Twist, self._cmd_vel_callback)
        self._speedin_sub = rospy.Subscriber("HALSpeedIn", HALSpeedIn, self._speedin_cb, queue_size=1)
        self._positionin_sub = rospy.Subscriber("HALPositionIn", HALPositionIn, self._positionin_cb, queue_size=1)
        self._headingin_sub = rospy.Subscriber("HALHeadingIn", HALHeadingIn, self._headingin_cb, queue_size=1)

        self.fake = False
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0
        self.dx = 0
        self.dr = 0
        self.then = rospy.Time.now()    # time for determining dx/dy

        self.base_frame_id = 'base_footprint'
        self.odom_frame_id = 'odom'

        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=5)
        self.odomBroadcaster = TransformBroadcaster()
        self._speedout_pub = HALSpeedOutPublisher()


    def _speedin_cb(self, msg):
        if not self.fake:
            self.dx = msg.linear
            self.dr = msg.angular

    def _positionin_cb(self, msg):
        if not self.fake:
            self.x = msg.x
            self.y = msg.y

    def _headingin_cb(self, msg):
        if not self.fake:
            self.th = msg.heading

    def _cmd_vel_callback(self, msg):
        self.last_cmd = rospy.Time.now()
        self.dx = msg.linear.x        # m/s
        self.dr = msg.angular.z       # rad/s
        self._speedout_pub.publish(SpeedData(linear=self.dx, angular=self.dr))

    def start(self):
        if self.fake:
            self._speedout_pub.publish(SpeedData(linear=0.0, angular=0.0))

    def loop(self):
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if now > self.t_next:

                if self.fake:
                    elapsed = now - self.then
                    self.then = now
                    elapsed = elapsed.to_sec()

                    x = cos(self.th)*self.dx*elapsed
                    y = -sin(self.th)*self.dx*elapsed
                    self.x += cos(self.th)*self.dx*elapsed
                    self.y += sin(self.th)*self.dx*elapsed
                    self.th += self.dr*elapsed

                # Update odometry and broadcast
                quaternion = Quaternion()
                quaternion.x = 0.0
                quaternion.y = 0.0
                quaternion.z = sin(self.th/2)
                quaternion.w = cos(self.th/2)
                self.odomBroadcaster.sendTransform(
                    (self.x, self.y, 0),
                    (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                    rospy.Time.now(),
                    self.base_frame_id,
                    self.odom_frame_id
                    )

                odom = Odometry()
                odom.header.stamp = now
                odom.header.frame_id = self.odom_frame_id
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.position.z = 0
                odom.pose.pose.orientation = quaternion
                odom.child_frame_id = self.base_frame_id
                odom.twist.twist.linear.x = self.dx
                odom.twist.twist.linear.y = 0
                odom.twist.twist.angular.z = self.dr
                self.odomPub.publish(odom)

                self.t_next = now + self.t_delta


    def shutdown(self):
        if self.fake:
            self._speedout_pub.publish(SpeedData(linear=0.0, angular=0.0))


if __name__ == "__main__":
    try:
        fakenode = FakeArlobotNode()
    except FakeArlobotNodeError as err:
        rospy.fatal("Unable to instantiate DriveNode - {}".format(err))
    else:
        try:
            fakenode.start()
            fakenode.loop()
        except rospy.ROSInterruptException as err:
            fakenode.shutdown()
            rospy.fatal("Interrupt Exception raised in DriveNode - {}".format(err))

# --- EOF ---

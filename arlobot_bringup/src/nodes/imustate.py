
# Third-party
import rospy

# Project
from arlobot_bringup.msg import (
    HALAngularVelocityIn,
    HALLinearAccelIn,
    HALOrientationIn
)
from pubs.imupub import ImuPublisher


class ImuState(object):
    """
    Helper class to consolidate publishing/subscribing and hold IMU state
    """
    def __str__(self):
        return "angvel: {}, linaccel: {}, orient: {}".format(self._angular_vel, self._linear_accel, self._orientation)

    def __init__(self):
        self._orientation = HALOrientationIn()
        self._angular_vel = HALAngularVelocityIn()
        self._linear_accel = HALLinearAccelIn()

        # Publishers
        # Note: The sensor node aggregates all sensors including the Imu.  Data from the Imu is used by a many different
        # nodes for different purposes.  The sensor node is publishing orientation, linear acceleration and angular
        # velocity.
        self._imu_pub = ImuPublisher()

        # Subscribers
        # Note: Subscribers come last to prevent callbacks before instance attributes
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

    def _angvelin_cb(self, msg):
        self.angular_vel = msg

    def _linaccel_cb(self, msg):
        self.linear_accel = msg

    def _orientin_cb(self, msg):
        self.orientation = msg

    def publish(self):
        self._imu_pub.publish(self._orientation,
                              self._linear_accel,
                              self._angular_vel)

# --- EOF ---

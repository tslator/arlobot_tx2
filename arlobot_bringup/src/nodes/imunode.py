#!/usr/bin/env python


# Standard
# None

# Third-Party
import rospy


# Project
from basenode import BaseNode
from arlobot_bringup.msg import (
    HALAngularVelocityIn,
    HALLinearAccelIn,
    HALOrientationIn
)
from pubs.imupub import ImuPublisher


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


class ImuNodeError(Exception):
    pass


class ImuNode(BaseNode):
    def __init__(self, debug=False):
        super(ImuNode, self).__init__(name='ImuNode', debug=debug)

        rospy.loginfo("ImuNode init")

        # Publishers
        self._imu_pub = ImuPublisher(frame_id='map')

        self._rate = rospy.Rate(20)
        self._imu_state = ImuState()

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
        if self._imu_state:
            self._imu_state.angular_velocity = msg

    def _linaccel_cb(self, msg):
        if self._imu_state:
            self._imu_state.linear_accel = msg

    def _orientin_cb(self, msg):
        if self._imu_state:
            self._imu_state.orientation = msg

    def start(self):
        """
        Performs operations that must happen before the main loop starts but after __init__
        """
        rospy.loginfo("ImuNode starting ...")
        rospy.loginfo("ImuNode started")

    def loop(self):
        """
        Runs the node main loop
        """
        rospy.loginfo("ImuNode starting loop ...")
        while not rospy.is_shutdown():

            
            if self._imu_state.is_ready:
                self._imu_pub.publish(self._imu_state.orientation, 
                                      self._imu_state.linear_accel,
                                      self._imu_state.angular_velocity)

            self._rate.sleep()

        self.shutdown()
        rospy.loginfo("ImuNode exiting loop")

    def shutdown(self):
        """
        Performs shutdown on any resources acquired
        """
        rospy.loginfo("ImuNode shutdown")

if __name__ == "__main__":
    try:
        imunode = ImuNode()

    except ImuNodeError as err:
        rospy.fatal("Unable to instantiate ImuNode - {}".format(err))
    else:
        try:
            imunode.start()
            imunode.loop()
        except rospy.ROSInterruptException as err:
            rospy.fatal("Interrupt Exception raised in ImuNode - {}".format(err))
            imunode.shutdown()
    


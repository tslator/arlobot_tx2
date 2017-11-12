#!/usr/bin/env python


# Standard
# None

# Third-Party
import rospy


# Project
from basenode import BaseNode
from imustate import ImuState


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
        self._imu_state.wait_for_message()
        rospy.loginfo("ImuNode started")

    def loop(self):
        """
        Runs the node main loop
        """
        rospy.loginfo("ImuNode starting loop ...")
        while not rospy.is_shutdown():
            self._imu_pub.publish()
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
    
# --- EOF ---

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

        self._rate = rospy.Rate(20)
        self._imu_state = ImuState()

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
            self._imu_state.publish()
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

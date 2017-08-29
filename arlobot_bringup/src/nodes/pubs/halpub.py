#! /usr/bin/env python
"""
This module provides classes for publishing the HAL messages.
"""
from __future__ import print_function

"""
---------------------------------------------------------------------------------------------------
Imports
---------------------------------------------------------------------------------------------------
"""

# Standard Library
# None

# Third-Party
# None

# Project
import rospy
from std_msgs.msg import Header
from arlobot_bringup.msg import (
    # PsocHw Messages
    HALStatusIn, HALSpeedIn, HALPositionIn, HALHeadingIn, HALHeartbeatIn, HALSpeedOut, HALControlOut,
    # ImuHw Messages
    HALOrientationIn, HALEulerIn, HALLinearAccelIn, HALAngularVelocityIn, HALMagneticIn, HALTempIn,
    # SensorHw Messages
    HALSensorArrayIn
)

"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

#---------------------------------------------------------------------------------------------------
# Publisher Base Class
#---------------------------------------------------------------------------------------------------

class HALPublisherBase(object):
    """
    Base class for publishing HAL messages.  Provides publisher creating and publishing.
    """
    __FMT_STR = "Publishing {}:"

    def __init__(self, msg_str, msg_type, frame_id='/base_link', queue_size=1):
        self._publisher = rospy.Publisher(msg_str, msg_type, queue_size=queue_size)
        self._msg_str = msg_str
        self._msg_type = msg_type
        self._frame_id = frame_id

    def _msg(self, data):
        """
        Method for creating specific messages.  To be overridden by base classes
        :param data: the data (named tuple) to inserted into the message
        :return: instantiated message with data
        """
        pass

    def publish(self, data, log=False):
        msg = self._msg(data)
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._frame_id

        self._publisher.publish(msg)

        if log:
            rospy.loginfo("{}\n{}".format(HALPublisherBase.__FMT_STR.format(self._msg_str), msg))

# ---------------------------------------------------------------------------------------------------
# PsocHw Publishers
#---------------------------------------------------------------------------------------------------

class HALStatusInPublisher(HALPublisherBase):
    """
    Publisher for HALStatusIn message
    """
    def __init__(self):
        super(HALStatusInPublisher, self).__init__('HALStatusIn', HALStatusIn)

    def _msg(self, data):
        return self._msg_type(device=data.device, calibration=data.calibration)


class HALSpeedInPublisher(HALPublisherBase):
    """
    Publisher for HALSpeedIn message
    """
    def __init__(self):
        super(HALSpeedInPublisher, self).__init__('HALSpeedIn', HALSpeedIn)

    def _msg(self, data):
        return self._msg_type(linear=data.linear, angular=data.angular)


class HALPositionInPublisher(HALPublisherBase):
    """
    Publisher for HALSpeedIn message
    """
    def __init__(self):
        super(HALPositionInPublisher, self).__init__('HALPositionIn', HALPositionIn)

    def _msg(self, data):
        return self._msg_type(x = data.x, y = data.y)


class HALHeadingInPublisher(HALPublisherBase):
    """
    Publisher for HALSpeedIn message
    """
    def __init__(self):
        super(HALHeadingInPublisher, self).__init__('HALHeadingIn', HALHeadingIn)

    def _msg(self, data):
        return self._msg_type(heading = data.heading)


class HALHeartbeatInPublisher(HALPublisherBase):
    """
    Publisher for HALSpeedIn message
    """
    def __init__(self):
        super(HALHeartbeatInPublisher, self).__init__('HALHeartbeatIn', HALHeartbeatIn)

    def _msg(self, data):
        return self._msg_type(heartbeat = data.heartbeat)


class HALSpeedOutPublisher(HALPublisherBase):
    """
    Publisher for HALSpeedOut message
    """
    def __init__(self):
        super(HALSpeedOutPublisher, self).__init__('HALSpeedOut', HALSpeedOut)

    def _msg(self, data):
        return HALSpeedOut(linear=data.linear, angular=data.angular)


class HALControlOutPublisher(HALPublisherBase):
    """
    Publisher for HALControlOut message
    """
    def __init__(self):
        super(HALControlOutPublisher, self).__init__('HALControlOut', HALControlOut)

    def _msg(self, data):
        return HALControlOut(device=data.device, debug=data.debug)


# ---------------------------------------------------------------------------------------------------
# ImuHw Publishers
#---------------------------------------------------------------------------------------------------

class HALOrientationInPublisher(HALPublisherBase):
    """
    Publisher for HALOrientationIn message
    """
    def __init__(self):
        super(HALOrientationInPublisher, self).__init__('HALOrientationIn', HALOrientationIn)

    def _msg(self, data):
        return self._msg_type(x = data.x, y = data.y, z = data.z, w = data.w)


class HALLinearAccelInPublisher(HALPublisherBase):
    """
    Publisher for HALLinearAccelIn message
    """
    def __init__(self):
        super(HALLinearAccelInPublisher, self).__init__('HALLinearAccelIn', HALLinearAccelIn)

    def _msg(self, data):
        return self._msg_type(x = data.x, y = data.y, z = data.z)


class HALAngularVelocityInPublisher(HALPublisherBase):
    """
    Publisher for HALAngularVelocityIn message
    """
    def __init__(self):
        super(HALAngularVelocityInPublisher, self).__init__('HALAngularVelocityIn', HALAngularVelocityIn)

    def _msg(self, data):
        return self._msg_type(x = data.x, y = data.y, z = data.z)


class HALMagneticInPublisher(HALPublisherBase):
    """
    Publisher for HALMagneticIn message
    """
    def __init__(self):
        super(HALMagneticInPublisher, self).__init__('HALMagneticIn', HALMagneticIn)

    def _msg(self, data):
        return self._msg_type(x = data.x, y = data.y, z = data.z)


class HALEulerInPublisher(HALPublisherBase):
    """
    Publisher for HALEulerIn message
    """
    def __init__(self):
        super(HALEulerInPublisher, self).__init__('HALEulerIn', HALEulerIn)

    def _msg(self, data):
        return self._msg_type(yaw = data.yaw, roll = data.roll, pitch = data.pitch)


class HALTempInPublisher(HALPublisherBase):
    """
    Publisher for HALTempIn message
    """
    def __init__(self):
        super(HALTempInPublisher, self).__init__('HALTempIn', HALTempIn)

    def _msg(self, data):
        return self._msg_type(fahrenheit = data.f, celcius = data.c)


# ---------------------------------------------------------------------------------------------------
# SensorHw Publishers
#---------------------------------------------------------------------------------------------------

class HALSensorArrayInPublisher(HALPublisherBase):
    """
    Publisher for HALSensorArrayIn message
    """
    def __init__(self):
        super(HALSensorArrayInPublisher, self).__init__('HALSensorArrayIn', HALSensorArrayIn)

    def _msg(self, data):
        return self._msg_type(source = data.source, data = data.data)


"""
---------------------------------------------------------------------------------------------------
Testing
---------------------------------------------------------------------------------------------------
"""


def module_test():
    print(__name__, "Module Test")


if __name__ == "__main__":
    module_test()

# --- EOF ---

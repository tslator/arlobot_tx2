#! /usr/bin/env python
"""
This module provides an interface for reading and writing to the i2c bus via the smbus module
It creates an abstraction that supports reading/writing numeric types and arrays of
numeric types.
"""
from __future__ import print_function

"""
---------------------------------------------------------------------------------------------------
Imports
---------------------------------------------------------------------------------------------------
"""
# Standard Library
# None

# Third Party
# None

# PRoject
from .basedevice import BaseDevice
from ..messages import Message, MSG_ID_ULTRASONIC_FRONT, MSG_ID_ULTRASONIC_REAR, MSG_ID_INFRARED_FRONT, MSG_ID_INFRARED_REAR


class SensorDeviceError(Exception):
    pass


class SensorCanDevice(BaseDevice):

    _ARB_ID_ULTRASONIC_FRONT = 0x201
    _ARB_ID_ULTRASONIC_REAR = 0x202
    _ARB_ID_INFRARED_FRONT = 0x203
    _ARB_ID_INFRARED_REAR = 0x204

    def __init__(self, name, device=None, logger=None):
        super(SensorCanDevice, self).__init__(name)

        self._device = device

        self._map_arbid_to_msgid = {
            self._ARB_ID_ULTRASONIC_FRONT : lambda data: Message(type=MSG_ID_ULTRASONIC_FRONT, data=data),
            self._ARB_ID_ULTRASONIC_REAR : lambda data: Message(type=MSG_ID_ULTRASONIC_REAR, data=data),
            self._ARB_ID_INFRARED_FRONT : lambda data: Message(type=MSG_ID_INFRARED_FRONT, data=data),
            self._ARB_ID_INFRARED_REAR : lambda data: Message(type=MSG_ID_INFRARED_REAR, data=data)
        }

    def recv(self):
        """
        :return:
        """
        msg = self._device.recv()
        return self._map_arbid_to_msgid[msg.arbitration_id](msg.data)

    def send(self, msg):
        pass

    def shutdown(self):
        # Note: The can interface supports a shutdown method, so it is exposed to the SensorDriver to be called on
        # driver stop (which is only called when the upper software layers are being shut down)
        self._device.shutdown()


class SensorMockDevice(BaseDevice):
    def __init__(self, name, device=None):
        self._device = device

    def recv(self):
        return self._device.recv()

    def send(self, msg):
        self._device.send(msg)


def module_test():
    pass

#--- EOF ---
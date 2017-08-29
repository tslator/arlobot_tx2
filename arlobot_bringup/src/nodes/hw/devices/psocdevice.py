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
import struct

# Third Party
import can
import rospy

# Project
from .basedevice import BaseDevice
from .mockdevice import MockDevice
from ..messages import Message, MSG_ID_CONTROL, MSG_ID_DEBUG, MSG_ID_SPEED, MSG_ID_STATUS, MSG_ID_POSITION, \
    MSG_ID_HEADING, MSG_ID_HEARTBEAT, StatusData, SpeedData, PositionData, HeadingData, HeartbeatData
from .i2cdevice import I2CDeviceError
from utils.logger import Logger

"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

class PsocDeviceError(Exception):
    pass


class PsocI2CDevice(BaseDevice):

    _I2C_ADDRESS = 0x08
    _RECV_OFFSET = 12
    _RECV_NUM_BYTES = 28
    _CONTROL_OFFSET = 0
    _DEBUG_OFFSET = 2
    _LEFT_RIGHT_SPEED_OFFSET = 4

    def __init__(self, name, device=None, logger=None):
        super(PsocI2CDevice, self).__init__(name)

        self._device = device or MockDevice('mock device')
        self._logger = logger or Logger()

        self._map_id_to_offset = {
            MSG_ID_CONTROL : ('H', self._CONTROL_OFFSET),
            MSG_ID_DEBUG : ('H', self._DEBUG_OFFSET),
            MSG_ID_SPEED : ('f', self._LEFT_RIGHT_SPEED_OFFSET)
        }

        # Slices that define the grouping of data for each message from the raw data fields
        STATUS = slice(0, 2)
        SPEED = slice(2, 4)
        POSITION = slice(4, 6)
        HEADING = 6
        HEARTBEAT = 7

        self._msg_data_slices = [STATUS, SPEED, POSITION, HEADING, HEARTBEAT]

        self._msgs = [
            lambda data: Message(type=MSG_ID_STATUS, data=StatusData(*data)),
            lambda data: Message(type=MSG_ID_SPEED, data=SpeedData(*data)),
            lambda data: Message(type=MSG_ID_POSITION, data=PositionData(*data)),
            lambda data: Message(type=MSG_ID_HEADING, data=HeadingData(data)),
            lambda data: Message(type=MSG_ID_HEARTBEAT, data=HeartbeatData(data))
        ]
   
        self._msg_gen = None

    def _create_msg_gen(self):
        try:
            data = self._device.read_array(self._I2C_ADDRESS, self._RECV_OFFSET, self._RECV_NUM_BYTES, 'B')
        except I2CDeviceError as err:
            rospy.logwarn("I2CDeviceError: ({}) - returning safe data".format(err))
            data = [0]*self._RECV_NUM_BYTES 
        finally:
            fields = struct.unpack("<HHfffffL", str(bytearray(data)))
            msg_data = [fields[s] for s in self._msg_data_slices]
            self._msg_gen = (self._msgs[i](msg_data[i]) for i in range(len(msg_data)))

    def _next_msg(self):
        if self._msg_gen is None:
            self._create_msg_gen()

        return next(self._msg_gen)

    def recv(self):
        try:
            return self._next_msg()
        except StopIteration:
            self._msg_gen = None
            return self._next_msg()

    def send(self, msg):
        try:
            type, offset = self._map_id_to_offset[msg.type]
            self._device.write_array(self._I2C_ADDRESS, offset, msg.data, type)
        except KeyError as err:
            self._logger.warn("Unknown key {} for mapping - {}".format(msg.type, err))


class PsocCANDevice(BaseDevice):

    _ARB_ID_CONTROL = 0x101
    _ARB_ID_DEBUG = 0x102
    _ARB_ID_SPEED = 0x103
    _ARB_ID_STATUS = 0x104
    _ARB_ID_DISTANCE = 0x105
    _ARB_ID_HEADING = 0x106
    _ARB_ID_HEARTBEAT = 0x107

    def __init__(self, name, device=None, logger=None):
        super(PsocCANDevice, self).__init__(name)

        self._device = device

        self._map_msgid_to_arbid = {
            MSG_ID_CONTROL : lambda data: can.Message(arbitration_id=self._ARB_ID_CONTROL, data=data),
            MSG_ID_DEBUG : lambda data: can.Message(arbitration_id=self._ARB_ID_DEBUG, data=data),
            MSG_ID_SPEED : lambda data: can.Message(arbitration_id=self._ARB_ID_SPEED, data=data)
        }

        self._map_arbid_to_msgid = {
            self._ARB_ID_STATUS : lambda data: Message(type=MSG_ID_STATUS, data=data),
            self._ARB_ID_SPEED : lambda data: Message(type=MSG_ID_SPEED, data=data),
            self._ARB_ID_DISTANCE : lambda data: Message(type=MSG_ID_POSITION, data=data),
            self._ARB_ID_HEADING : lambda data: Message(type=MSG_ID_HEADING, data=data),
            self._ARB_ID_HEARTBEAT : lambda data: Message(type=MSG_ID_HEARTBEAT, data=data)
        }

    def recv(self):
        """
        Read all data from i2c bus (status, speed, distance, heading, heartbeat - 26 bytes)
        parse the data into the respective messages
        yield each message
        Rinse and Repeat
        :return: 
        """
        can_msg = self._device.recv()
        return self._map_arbid_to_msgid[can_msg.arbitration_id](can_msg.data)

    def send(self, msg):
        # Use the message type to obtain an offset and method for writing data
        # Execute the method with the address, offset and data
        self._device.send(self._map_msgid_to_arbid[msg.type])


class PsocMockDevice(BaseDevice):
    def __init__(self, name, device=None, logger=None):
        self._device = device

    def recv(self):
        return self._device.recv()

    def send(self, msg):
        self._device.send(msg)


def module_test():
    from i2cdevice import I2CDevice
    import time
    logger = Logger()

    logger.debug("Performing module test")

    try:
        i2c_device = I2CDevice(device=I2CDevice.DEV_I2C_1, logger=logger)
    except I2CDeviceError as err:
        logger.fatal("Unable to instantiate Psoc hardware", err)

    psoc_device = PsocI2CDevice(name='psoc device', device=i2c_device)

    for i in range(10):
        msg = psoc_device.recv()
        logger.debug(msg)
        time.sleep(1)


if __name__ == "__main__":
    module_test()

#--- EOF ---

#!/usr/bin/env python
"""
---------------------------------------------------------------------------------------------------
File: imudriver.py

Description: This module contains class definitions for the IMUDriver

Author: Tim Slator
Date: 11APR17
License: MIT
---------------------------------------------------------------------------------------------------
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

# Project
from .devicedriver import DeviceDriver
from utils.logger import Logger


"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

class SensorDriverError(Exception):
    pass


class SensorDriver(DeviceDriver):
    def __init__(self, name, device=None, logger=None):
        self._name = name
        self._device = device
        self._hw_callback = None
        self._logger = logger or Logger()

        super(SensorDriver, self).__init__(name, self._device, self._process, logger=logger)

    def _process(self, msg):
        self._hw_callback(msg)

    def start(self, callback):
        self._hw_callback = callback
        super(SensorDriver, self).start()

    def send(self, msg):
        """
        Type = text indicating the type of message
        Data = namedtuple of the data
        :param msg:
        :return:
        """
        self._device.send(msg)

    def stop(self):
        self._device.shutdown()
        super(SensorDriver, self).stop()

def module_test():
    '''
    from ..devices.candevice import CANDevice
    from ..devices.i2cdevice import I2CDevice
    from .candriver import CANDriver
    from .i2cdriver import I2CDriver
    '''
    print(__name__, " Module Test")


if __name__ == "__main__":
    module_test()


#--- EOF ---

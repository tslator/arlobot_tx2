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
import rospy

# Project
from .devicedriver import DeviceDriver
from utils.logger import Logger

"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

class PsocDriverError(Exception):
    pass


class PsocDriver(DeviceDriver):
    def __init__(self, name, device=None, logger=None):
        self._name = name
        self._device = device
        self._hw_callback = None

        super(PsocDriver, self).__init__(name, self._device, self._process)

    def _process(self, msg):
        self._hw_callback(msg)

    def start(self, callback):
        self._hw_callback = callback
        super(PsocDriver, self).start()

    def send(self, msg):
        """
        Type = text indicating the type of message
        Data = namedtuple of the data
        :param msg: 
        :return: 
        """
        self._device.send(msg)


def module_test():
    import time
    from ..devices.i2cdevice import I2CDevice
    from ..devices.psocdevice import PsocI2CDevice

    def callback(msg):
        logger.debug("{} - {}".format(time.time(), msg))

    logger = Logger()

    logger.debug("{} {}".format(__name__, " Module Test"))

    i2c_device = I2CDevice(device=I2CDevice.DEV_I2C_1)
    device = PsocI2CDevice(name='psoc device', device=i2c_device)
    driver = PsocDriver(name='psochw i2c driver', device=device)

    driver.start(callback)
    time.sleep(5)
    driver.stop()

if __name__ == "__main__":
    module_test()

#--- EOF ---

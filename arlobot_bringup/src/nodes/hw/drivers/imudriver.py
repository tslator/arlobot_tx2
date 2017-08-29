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

# Standard
#None

# Third-Party
import rospy

# Project
from devicedriver import DeviceDriver

"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

#CalibrationStatus = namedtuple('CalibrationStatus', 'sys gyro accel mag')


class ImuDriverError(Exception):
    pass


class ImuDriver(DeviceDriver):

    def __init__(self, name, device=None, logger=None):
        self._hw_callback = None
        self._device = device #or MockImuDevice(i2c_dev=MockI2CDevice(I2CDevice.DEV_I2C_1))
        self._logger = logger
        super(ImuDriver, self).__init__(name, self._device, self._process)

    def _process(self, msg):
        self._hw_callback(msg)

    def start(self, callback):
        self._hw_callback = callback
        super(ImuDriver, self).start()

    def get_status(self):
        """ returns the calibration status of the imu"""
        #status = self._device.get_calibration_status()
        #
        #return CalibrationStatus(*status)
        pass


def module_test():
    print(__name__, " Module Test")

# --- EOF ---

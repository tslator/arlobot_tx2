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
# None

# Third-Party
import can

# Project
from .sensorhw import SensorHw, SensorHwError
from devices.sensordevice import SensorCanDevice, SensorDeviceError
from drivers.sensordriver import SensorDriver, SensorDriverError
from common import Logger

"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""


class SensorHwFactoryError(Exception):
    pass

class SensorHwFactory:
    _DEFAULT_CHANNEL = '/dev/can0'
    _DEFAULT_BUSTYPE = 'socketcan'

    @classmethod
    def create_can(cls, driver_cb, channel=_DEFAULT_CHANNEL, bustype=_DEFAULT_BUSTYPE, logger=None):
        if not logger:
            logger = Logger()

        try:
            can_device = can.interface.Bus(channel=channel, bustype=bustype)
        except can.CanError as err:
            raise SensorHwFactoryError("Unable to instantiate Sensor hardware", err)

        try:
            device = SensorCanDevice(name='sensor device', device=can_device, logger=logger)
        except SensorDeviceError as err:
            raise SensorHwFactoryError("Unable to instantiate Sensor hardware", err)

        try:
            driver = SensorDriver(name='Sensor driver', device=device, logger=logger)
        except SensorDriverError as err:
            raise SensorHwFactoryError("Unable to instantiate Sensor hardware", err)

        try:
            return SensorHw(driver_if=driver, driver_cb=driver_cb, logger=logger)
        except SensorHwError as err:
            raise SensorHwFactoryError("Unable to instantiate Sensor hardware", err)


def module_test():
    pass

if __name__ == "__main__":
    module_test()


#--- EOF ---

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
import sys

# Third-Party
import can

# Project
from .psochw import PsocHw, PsocHwError
from drivers.psocdriver import PsocDriver, PsocDriverError
from devices.i2cdevice import I2CDevice, I2CDeviceError
from devices.psocdevice import PsocCANDevice, PsocI2CDevice, PsocMockDevice, PsocDeviceError
from devices.mockdevice import MockDevice, MockDeviceError
from utils.logger import Logger


"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

class PsocHwFactoryError(Exception):
    pass

class PsocHwFactory:

    _DEFAULT_CHANNEL = '/dev/can0'
    _DEFAULT_BUSTYPE = 'socketcan'

    @classmethod
    def create_can(cls, driver_cb, channel=_DEFAULT_CHANNEL, bustype=_DEFAULT_BUSTYPE, logger=None):
        if not logger:
            logger = Logger()

        try:
            can_device = can.interface.Bus(channel=channel, bustype=bustype)
        except can.CanError as err:
            raise PsocHwFactoryError("Unable to instantiate Psoc hardware", err)

        try:
            device = PsocCANDevice(name='psoc device', device=can_device, logger=logger)
        except PsocDeviceError as err:
            raise PsocHwFactoryError("Unable to instantiate Psoc hardware", err)

        try:
            driver = PsocDriver(name='psoc driver', device=device, logger=logger)
        except PsocDriverError as err:
            raise PsocHwFactoryError("Unable to instantiate Psoc hardware", err)

        try:
            return PsocHw(driver_if=driver, driver_cb=driver_cb, logger=logger)
        except PsocHwError as err:
            raise PsocHwFactoryError("Unable to instantiate Psoc hardware", err)

    @classmethod
    def create_i2c(cls, driver_cb, logger=None):
        if not logger:
            logger = Logger()

        logger.debug("({}) - Creating I2CDevice ...".format(__name__))
        try:
            i2c_device = I2CDevice(device=I2CDevice.DEV_I2C_1, logger=logger)
        except I2CDeviceError as err:
            raise PsocHwFactoryError("Unable to instantiate Psoc hardware", err)
        logger.debug("({}) - Complete".format(__name__))

        logger.debug("({}) - Creating PsocI2CDevice ...".format(__name__))
        try:
            device = PsocI2CDevice(name='psoc device', device=i2c_device, logger=logger)
        except PsocDeviceError as err:
            raise PsocHwFactoryError("Unable to instantiate Psoc hardware", err)
        logger.debug("({}) - Complete".format(__name__))

        logger.debug("({}) - Creating PsocDriver ...".format(__name__))
        try:
            driver = PsocDriver(name='psochw i2c driver', device=device, logger=logger)
        except PsocDriverError as err:
            raise PsocHwFactoryError("Unable to instantiate Psoc hardware", err)
        logger.debug("({}) - Complete".format(__name__))

        logger.debug("({}) - Creating PsocHw ...".format(__name__))
        try:
            return PsocHw(driver_if=driver, driver_cb=driver_cb, logger=logger)
        except PsocHwError as err:
            raise PsocHwFactoryError("Unable to instantiate Psoc hardware", err)
        logger.debug("({}) - Complete".format(__name__))

    @classmethod
    def create_mock(cls, driver_cb, logger=None):
        if not logger:
            logger = Logger()

        try:
            mock_device = MockDevice('mock device', logger=logger)
        except MockDeviceError as err:
            raise PsocHwFactoryError("Unable to instantiate Psoc hardware", err)

        try:
            device = PsocMockDevice('psoc device', device=mock_device, logger=logger)
        except PsocDeviceError as err:
            raise PsocHwFactoryError("Unable to instantiate Psoc hardware", err)

        try:
            driver = PsocDriver('module i2c driver', device=device, logger=logger)
        except PsocDriverError as err:
            raise PsocHwFactoryError("Unable to instantiate Psoc hardware", err)

        try:
            return PsocHw(driver_if=driver, driver_cb=driver_cb, logger=logger)
        except PsocHwError as err:
            raise PsocHwFactoryError("Unable to instantiate Psoc hardware", err)


def module_test(selection):

    def callback(msg):
        logger.debug(msg)

    logger = Logger()
    if selection == 'can':
        pass
    elif selection == 'i2c':
        PsocHwFactory.create_i2c(callback, logger)
    elif selection == 'mock':
        pass

if __name__ == "__main__":
    #module_test(sys.argv[1])
    module_test('i2c')


#--- EOF ---

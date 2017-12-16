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
# None

# Project
from devices.i2cdevice import I2CDevice, I2CDeviceError
from devices.imudevice import ImuDevice, ImuDeviceError
from drivers.imudriver import ImuDriver, ImuDriverError
from imuhw import ImuHw, ImuHwError
from common import Logger

"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""


class ImuHwFactoryError(Exception):
    pass


class ImuHwFactory(object):
    @classmethod
    def create_i2c(cls, driver_cb, logger=None):
        if not logger:
            logger = Logger()

        try:
            i2c_device = I2CDevice(I2CDevice.DEV_I2C_1, logger=logger)
        except I2CDeviceError as err:
            raise ImuHwFactoryError("Unable to instantiate Imu hardware", err)

        try:
            imu_device = ImuDevice(name="imu device", device=i2c_device, logger=logger)
        except ImuDeviceError as err:
            raise ImuHwFactoryError("Unable to instantiate Imu hardware", err)

        try:
            driver = ImuDriver('imu driver', device=imu_device, logger=logger)
        except ImuDriverError as err:
            raise ImuHwFactoryError("Unable to instantiate Imu hardware", err)

        try:
            return ImuHw(driver=driver, callback=driver_cb, logger=logger)
        except ImuHwError as err:
            raise ImuHwFactoryError("Unable to instantiate Imu hardware", err)


def module_test():
    import time
    from imuhwfactory import ImuHwFactory
    def callback(msg):
        logger.info("delivered callback msg: {}".format(msg))

    logger = Logger()
    print("ImuHwFactory.create_i2c ...")
    imuhw = ImuHwFactory.create_i2c(callback, logger=logger)
    print("ImuHwFactory.create_i2c complete")
    time.sleep(1)
    imuhw.shutdown()


if __name__ == "__main__":
    module_test()

#--- EOF ---

#!/usr/bin/env python
"""
---------------------------------------------------------------------------------------------------
File: psochw.py

Description: Provides implementation for receiving from/sending to the Psoc Hardware component.
The Psoc Hardware component implements functionality for a differential drive. 
hardware devices.

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
import time

# Third-Party
import rospy

# Project
from utils.logger import Logger

"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

class ImuHwError(Exception):
    pass


class ImuHw(object):
    """
    The ImuHw wraps the IMU driver which can be implemented using various physical IMU hardware devices
    and translates IMU data into messages delivered to the HAL node.
    """
    def __init__(self, driver=None, callback=None, logger=None):

        # Instantiate the IMUDriver and start it
        self._driver = driver # or MockImuDriver()

        # Capture the callback for local access
        self._callback = callback

        self._logger = logger or Logger()

    def _imu_callback(self, msg):
        self._callback(msg)

    def start(self):
        self._driver.start(self._imu_callback)

    def shutdown(self):
        start = time.time()
        self._logger.info("ImuHw starting shutdown ...")
        self._driver.stop()
        self._logger.info("ImuHw shutdown time: {}".format(time.time() - start))
                
    def calibrate(self, data):
        self._logger.info("imuhw calibrating")
        self._driver.calibrate()

    def get_status(self):
        """ Returns imu system and calibration status """
        pass


def module_test():
    from imuhwfactory import ImuHwFactory

    def callback(msg):
        logger.debug(str(msg))

    logger = Logger()

    imuhw = ImuHwFactory.create_i2c(driver_cb=callback, logger=logger)
    imuhw.start()
    time.sleep(0.05)
    imuhw.shutdown()


if __name__ == "__main__":
    module_test()

# --- EOF ---

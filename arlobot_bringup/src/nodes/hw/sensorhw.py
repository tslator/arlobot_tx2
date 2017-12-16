#!/usr/bin/env python
"""
---------------------------------------------------------------------------------------------------
File: sensorhw.py

Description: Receives CAN messages containing sensor data and returned namedtuple messages

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

# Project
from common import Logger


NUM_FRONT_SENSORS = 8
NUM_REAR_SENSORS = 8

NUM_SENSORS = NUM_FRONT_SENSORS + NUM_REAR_SENSORS

SENSOR_DATA_FRONT = slice(0, NUM_FRONT_SENSORS)
SENSOR_DATA_REAR = slice(NUM_REAR_SENSORS, NUM_SENSORS)


"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

class SensorHwError(Exception):
    pass


class SensorHw(object):
    def __init__(self, driver_if=None, driver_cb=None, logger=None):

        if driver_if is not None:
            self._driver_if = driver_if # or MockSensorDriver()

        if driver_cb is not None:
            self._driver_cb = driver_cb

        self._logger = logger or Logger()

    def _driver_callback(self, msg):
        self._driver_cb(msg)

    def start(self):
        self._driver_if.start(self._driver_callback)

    def shutdown(self):
        start = time.time()
        self._logger.info("SensorHw starting shutdown ... ")

        if self._driver_if is not None:
            self._driver_if.stop()

        self._logger.info("SensorHw shutdown time: {}".format(time.time() - start))


def module_test():
    pass


#--- EOF ---

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
from .messages import Message, ControlData, DebugData, SpeedData, MSG_ID_SPEED, MSG_ID_CONTROL, MSG_ID_DEBUG
from common import Logger

"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""


class PsocHwError(Exception):
    pass


class PsocHw(object):
    def __init__(self, driver_if=None, driver_cb=None, debug_if=None, debug_cb=None, cal_if=None, cal_cb=None, logger=None):

        self._driver_if = driver_if #or MockPsocDriver()
        self._debug_if = debug_if
        self._cal_if = cal_if

        self._driver_cb = driver_cb
        self._debug_cb = debug_cb
        self._cal_cb = cal_cb
        self._logger = logger or Logger()

    def _drive_callback(self, msg):
        self._driver_cb(msg)

    def _debug_callback(self, msg):
        self._debug_cb(self, msg)

    def _cal_callback(self, msg):
        self._cal_cb(msg)

    def start(self):
        if self._driver_if is not None:
            self._driver_if.start(self._drive_callback)

        if self._debug_if is not None:
            self._debug_if.start(self._debug_cb)

        if self._cal_if is not None:
            self._cal_if.start(self._cal_cb)

    def shutdown(self):
        start = time.time()
        self._logger.info("PsocHw starting shutdown ...")

        if self._driver_if is not None:
            self._logger.debug("Stopping driver if ...")
            self._driver_if.stop()
            self._logger.debug("driver if stopped")
        if self._debug_if is not None:
            self._logger.debug("Stopping debug if ...")
            self._debug_if.stop()
            self._logger.debug("debug if stopped")
        if self._cal_if is not None:
            self._logger.debug("Stopping cal if ...")
            self._cal_if.stop()
            self._logger.debug("cal if stopped")

        self._logger.info("PsocHw shutdown time: {}".format(time.time() - start))

    def set_speed(self, left, right):
        self._logger.debug("PsocHw setting speed: {}-{}".format(left, right))
        self._driver_if.send(Message(type=MSG_ID_SPEED, data=SpeedData(left, right)))
        
    def set_control(self, data):
        self._logger.debug("PsocHw received sending control data: {}".format(data))
        self._driver_if.send(Message(type=MSG_ID_CONTROL, data=ControlData(*data)))

    def set_debug(self, data):
        self._logger.debug("PsocHw received sending control data: {}".format(data))
        self._driver_if.send(Message(type=MSG_ID_DEBUG, data=DebugData(*data)))


def module_test():
    from psochwfactory import PsocHwFactory
    def callback(msg):
        logger.debug("delivered callback msg: {}".format(msg))

    logger = Logger()

    logger.debug("{} {}".format(__name__, "Module Test"))

    #psochw = PsocHwFactory.create_can(callback)
    #time.sleep(1)
    #psochw.shutdown()

    psochw = PsocHwFactory.create_i2c(callback, logger=logger)
    logger.debug("Starting psochw ...")
    psochw.start()
    logger.debug("Started")
    time.sleep(0.01)
    logger.debug("Shutting down psochw ...")
    psochw.shutdown()
    logger.debug("Shutdown")

    #psochw = PsocHwFactory.create_mock(callback)
    #time.sleep(1)
    #psochw.shutdown()


if __name__ == "__main__":
    module_test()

# --- EOF ---

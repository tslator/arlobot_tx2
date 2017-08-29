#!/usr/bin/env python
"""
---------------------------------------------------------------------------------------------------
File: logger.py

Description: This module provides a wrapper around logging and rospy logging to allow lower layer
             modules to log with or without rospy depending on how the module is instantiated.

Author: Tim Slator
Date: 01AUG17
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
import logging
import functools

# Third-party
import rospy


"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""


class Logger:
    def __init__(self, type='logging', level='info'):

        self._type = type

        self._log_map = {
            'rospy': {
                'info': rospy.loginfo,
                'warn': rospy.logwarn,
                'debug': rospy.logdebug,
                'fatal': rospy.logfatal
            },
            'logging': {
                'info': functools.partial(logging.log, logging.INFO),
                'warn': functools.partial(logging.log, logging.WARNING),
                'debug': functools.partial(logging.log, logging.ERROR),
                'fatal': functools.partial(logging.log, logging.CRITICAL)

            }
        }

    def _log(self, msg, level='info'):
        try:
            self._log_map[self._type.lower()][level.lower()](msg)
        except KeyError as err:
            logging.fatal("Unknown logging type {}, level {}".format(self._type, level))

    def log(self, msg, level):
        self._log(msg, level)

    def info(self, msg):
        self._log(msg, 'info')

    def warn(self, msg):
        self._log(msg, 'warn')

    def debug(self, msg):
        self._log(msg, 'debug')

    def fatal(self, msg):
        self._log(msg, 'fatal')

#--- EOF ---
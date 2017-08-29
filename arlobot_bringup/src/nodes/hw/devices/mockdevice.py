#!/usr/bin/env python
"""
---------------------------------------------------------------------------------------------------
File: mockdevice.py

Description: This module contains class definitions for various devices.  The devices are 
instantiated by drivers.

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
from basedevice import BaseDevice

"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

class MockDeviceError(Exception):
    pass


class MockDevice(BaseDevice):
    """
    Fake device used for integration
    """
    def __init__(self, name, logger=None):
        super(MockDevice, self).__init__(name)

        self._device = 'open(name)'
        self._count = 0

    def send(self, data):
        print("{}-{}".format(self._name, data))
        
    def recv(self):
        msg = "mock-msg-{}".format(self._count)
        self._count += 1
        return "{}-{}".format(self._name, msg)


def module_test():
    dev = MockDevice("mock device")
    print("Sent: ", "1")
    dev.send("1")
    data = dev.recv()
    print("Recv: ", data)


# --- EOF ---

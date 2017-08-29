#!/usr/bin/env python
"""
---------------------------------------------------------------------------------------------------
File: devices.py

Description: This module contains class definitions for various devices.  The devices are 
instantiated by drivers.

Author: Tim Slator
Date: 11APR17
License: MIT
---------------------------------------------------------------------------------------------------
"""

"""
---------------------------------------------------------------------------------------------------
Imports
---------------------------------------------------------------------------------------------------
"""

# Standard
import abc

# Third-Party
# None

# Project
# None

"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

class BaseDevice():
    """
    Base device defining abstract methods for all devices
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, name):
        self._name = name

    @abc.abstractmethod
    def send(self):
        pass
        
    @abc.abstractmethod
    def recv(self):
        return

# --- EOF ---

#!/usr/bin/env python
from __future__ import print_function
"""
---------------------------------------------------------------------------------------------------
File: halnode.py

Description: Provides implementation of the HALNode which is the common access point for all 
hardware devices.

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
# None

# Third-Party
import rospy

# Project
from basenode import BaseNode
from hw.halhw import SimulatedHardware

"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

class HALNodeError(Exception):
    pass


class HALNode(BaseNode):
    """
    Hardware Abstraction Layer (HAL) node.  Exposes messages for sending and receiving data from
    hardware devices.
    """
    def __init__(self, hardware=None, debug=False):
        super(HALNode, self).__init__(name='HALNode', debug=debug)

        rospy.loginfo("HALNode init")

        self._hw = hardware or SimulatedHardware()
        self._rate = rospy.Rate(50)

    def start(self):
        """
        Performs operations that must happen before the main loop starts but after __init__
        """
        rospy.loginfo("HALNode starting ...")
        self._hw.start()
        rospy.loginfo("HALNode started")
        
    def loop(self):
        """
        Runs the node main loop
        """
        rospy.loginfo("HALNode starting loop ...")
        while not rospy.is_shutdown():
            self._rate.sleep()

        rospy.loginfo("HALNode exiting loop")
                    
    def shutdown(self):
        """
        Performs shutdown on any resources acquired
        """
        rospy.loginfo("HALNode shutdown")
        self._hw.shutdown();

# --- EOF ---

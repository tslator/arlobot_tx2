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
from halnode import HALNode
from hw.halhw import ArlobotHardware, SimulatedHardware

class HALFactory(object):
    @classmethod
    def create_arlobot_hal(cls, debug=False):
        return HALNode(hardware=ArlobotHardware(), debug=debug)

    @classmethod
    def create_simulated_hal(cls, debug=False):
        return HALNode(hardware=SimulatedHardware(), debug=debug)


if __name__ == "__main__":
    try:
        simulated = rospy.get_param('/simulated', False)
        debug = rospy.get_param('/debug', False)

        if simulated:
            halnode = HALFactory.create_simulated_hal(debug=debug)
        else:
            halnode = HALFactory.create_arlobot_hal(debug=debug)

    except HALNodeError as err:
        rospy.fatal("Unable to instantiate HALNode - {}".format(err))
    else:
        try:
            halnode.start()
            halnode.loop()
        except rospy.ROSInterruptException as err:
            rospy.fatal("Interrupt Exception raised in HALNode - {}".format(err))
            halnode.shutdown()

# --- EOF ---

#!/usr/bin/env python
from __future__ import print_function
"""
---------------------------------------------------------------------------------------------------
File: sensornodefactory.py

Description: Provides implementation of the SensorNodeFactory which instantiates the appropriate
SensorNode.

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
from sensornode import SensorNode


class SensorNodeFactory(object):
    @classmethod
    def create_sensor_node(cls, debug):
        return SensorNode(debug=debug)


if __name__ == "__main__":
    try:
        debug = rospy.get_param('/debug', False)                
        sensornode = SensorNodeFactory.create_sensor_node(debug=debug)
    except SensorNodeError as err:
        rospy.fatal("Unable to instantiate SensorNode - {}".format(err))
    else:
        try:
            sensornode.start()
            sensornode.loop()
        except rospy.ROSInterruptException as err:
            rospy.fatal("Interrupt Exception raised in sensornode - {}".format(err))
            sensornode.shutdown()

# --- EOF ---


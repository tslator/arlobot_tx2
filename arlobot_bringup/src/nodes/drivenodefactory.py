#!/usr/bin/env python
from __future__ import print_function
"""
---------------------------------------------------------------------------------------------------
File: drivenode.py

Description: Provides implementation of the DriveNode which handles all velocity commands and 
odometry publishing.

Author: Tim Slator
Date: 04NOV17
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

# Third Party
import rospy

# Project
from drivenode import DriveNode


"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""
class DriveNodeFactoryError(Exception):
    pass


class DriveNodeFactory:
    """
    Provide factory to create the drive node
    """

    @classmethod
    def create_drive_node(cls, debug=False):
        return DriveNode("DriveNode", debug)


if __name__ == "__main__":
    try:
        debug = rospy.get_param('/debug', False)        
        drivenode = DriveNodeFactory.create_drive_node(debug=debug)

    except DriveNodeFactoryError as err:
        rospy.fatal("Unable to instantiate DriveNode - {}".format(err))
    else:
        try:
            drivenode.start()
            drivenode.loop()
        except rospy.ROSInterruptException as err:
            drivenode.shutdown()
            rospy.fatal("Interrupt Exception raised in DriveNode - {}".format(err))

# --- EOF ---

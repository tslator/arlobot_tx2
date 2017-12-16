#!/usr/bin/env python
#--------------------------------------------------------------------------------------------------
# File: basenode.py
# Description: Provides a ABC base class for ROS nodes
# Author: Tim Slator
# Date: 11APR17
# License: MIT
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
# Imports
#--------------------------------------------------------------------------------------------------

# Standard
import abc

# ROS
import rospy


#--------------------------------------------------------------------------------------------------
# Classes
#--------------------------------------------------------------------------------------------------

class BaseNode(object):
    """
    Provides a base class implementation for all ROS nodes
    """
    __metaclass__ = abc.ABCMeta

    def __init__(self, name, debug):
        param_debug = rospy.get_param(name+'Debug', False)
        if debug or param_debug:
            rospy.init_node(name, anonymous=True, log_level=rospy.DEBUG)
        else:
            rospy.init_node(name, anonymous=True)
        rospy.on_shutdown(self.shutdown)

    @abc.abstractmethod
    def start(self):
        """
        Performs processing that needs to occur before the node enters its loop but that cannot be done in __init__
        """
        pass

    @abc.abstractmethod
    def loop(self):
        """
        Performs period operations as required by the node.
        """
        pass

    @abc.abstractmethod
    def shutdown(self):
        """
        Performs shutdown on any objects created during the lifecyle of the node
        """
        pass

# --- EOF ---
#!/usr/bin/env python
#--------------------------------------------------------------------------------------------------
# File: scansensorpub.py
#
# Description: Implements classes to publish LaserScan messages from ultrasonic and infrared 
#              range data.
#
# Author: Tim Slator
# Date: 11APR17
# License: MIT
#--------------------------------------------------------------------------------------------------
from __future__ import print_function

#--------------------------------------------------------------------------------------------------
# Imports
#--------------------------------------------------------------------------------------------------

# Standard
from math import cos, sin


# Third-Party
from rospy import Publisher
from sensor_msgs.msg import PointCloud, PointCloud2



class PointCloudPublisherError(Exception):
    pass


class PointCloudPublisher(object):
    def __init__(self):
        self._publisher = Publisher('PointCloud', PointCloud, queue_size=10)

    def _msg(self, nodes):
        pass

    def publish(self, nodes):
        pass

def module_test():
    print(__name__, "Module Test")

#--- EOF ---
#!/usr/bin/env python

# Third-Party
import rospy

# Project
from arlobot_bringup.msg import HALSensorArrayIn
from pubs.rangesensorpub import UltrasonicArrayPublisher, InfraredArrayPublisher
from pubs.scansensorpub import UltrasonicScanPublisher, InfraredScanPublisher


class SensorState(object):

    SOURCE_INFRARED = 'infrared'
    SOURCE_ULTRASONIC = 'ultrasonic'

    def __init__(self, source, array_pub=None, scan_pub=None):
        self._sensor_data = []
        self._source = source
        self._array_pub = array_pub
        self._scan_pub = scan_pub

        self._sub = rospy.Subscriber('HALSensorArrayIn',
                                     HALSensorArrayIn,
                                     self._callback)

    def _callback(self, msg):
        if msg.source == self._source:
            self._sensor_data = msg.data

    def publish(self):
        if self._array_pub is not None:
            self._array_pub.publish(self._sensor_data)
        if self._scan_pub is not None:
            self._scan_pub.publish(self._sensor_data)


class UltrasonicSensorState(SensorState):
    def __init__(self):
        super(UltrasonicSensorState, self).__init__(SensorState.SOURCE_ULTRASONIC,
                                                    UltrasonicArrayPublisher(),
                                                    UltrasonicScanPublisher())

class InfraredSensorState(SensorState):
    def __init__(self):
        super(InfraredSensorState, self).__init__(SensorState.SOURCE_INFRARED,
                                                  InfraredArrayPublisher(),
                                                  InfraredScanPublisher())


# --- EOF ---






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
from hw.psochwfactory import PsocHwFactory, PsocHwFactoryError
from hw.imuhwfactory import ImuHwFactory, ImuHwFactoryError
from hw.sensorhwfactory import SensorHwFactory, SensorHwFactoryError
from hw.sensorhw import NUM_SENSORS, SENSOR_DATA_FRONT, SENSOR_DATA_REAR
import hw.messages as hwmsg
from arlobot_bringup.msg import (
    # PsocHw Messages
    HALStatusIn, HALSpeedIn, HALPositionIn, HALHeadingIn, HALHeartbeatIn, HALSpeedOut, HALControlOut,
    # ImuHw Messages
    HALOrientationIn, HALLinearAccelIn, HALAngularVelocityIn, HALMagneticIn, HALEulerIn, HALTempIn,
    # SensorHw Messages
    HALSensorArrayIn
)
import pubs.halpub as halpub
from utils.logger import Logger

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
    def __init__(self, debug=False):
        super(HALNode, self).__init__(name='HALNode', debug=debug)

        rospy.loginfo("HALNode init")

        self._rate = rospy.Rate(50)

        self._ultrasonic_data = [0.0,] * NUM_SENSORS
        self._infrared_data = [0.0,] * NUM_SENSORS

        self._psoc_if_dict = {
            'i2c': PsocHwFactory.create_i2c(driver_cb=self._psochw_driver_cb, logger=Logger('rospy')),
            'can': PsocHwFactory.create_can(driver_cb=self._psochw_driver_cb, logger=Logger('rospy')),
            'mock': PsocHwFactory.create_mock(driver_cb=self._psochw_driver_cb, logger=Logger('rospy'))
        }

        psoc_interface = rospy.get_param('Psoc Interface', 'i2c')

        try:
            try:
                self._psochw = self._psoc_if_dict[psoc_interface]
            except PsocHwFactoryError as err:
                raise HALNodeError("Failure to instantiate PSOC hardware", err)
        except KeyError as err:
            raise HALNodeError("Unknown PSOC interface {}".format(psoc_interface), err)

        try:
            self._imuhw = ImuHwFactory.create_i2c(driver_cb=self._imuhw_callback, logger=Logger('rospy'))
        except ImuHwFactoryError as err:
            raise HALNodeError("Failure to instantiate IMU hardware", err)

        try:
            self._sensorhw = SensorHwFactory.create_can(driver_cb=self._sensorhw_callback, logger=Logger('rospy'))
        except SensorHwFactoryError as err:
            raise HALNodeError("Failure to instantiate Sensor hardware", err)

        '''
        ----------------------------------------------------------------------
        Subscribers for messages from other nodes to the HAL ('Out' messages)
        ----------------------------------------------------------------------
        '''
        self._hal_controlout_sub = rospy.Subscriber('HALControlOut', HALControlOut, self._hal_controloutmsg_callback,
                                                    queue_size=1)
        self._hal_speedout_sub = rospy.Subscriber('HALSpeedOut', HALSpeedOut, self._hal_speedoutmsg_callback,
                                                  queue_size=1)

        '''
        ----------------------------------------------------------------------
        Publishers for messages from the HAL to other nodes ('In' messages)
        ----------------------------------------------------------------------
        '''

        # PsocHw messages
        hal_statusin_pub = halpub.HALStatusInPublisher()
        hal_speedin_pub = halpub.HALSpeedInPublisher()
        hal_positionin_pub = halpub.HALPositionInPublisher()
        hal_headingin_pub = halpub.HALHeadingInPublisher()
        hal_heartbeatin_pub = halpub.HALHeartbeatInPublisher()

        # ImuHw messages
        hal_orientationin_pub = halpub.HALOrientationInPublisher()
        hal_linearaccelin_pub = halpub.HALLinearAccelInPublisher()
        hal_angularvelocityin_pub = halpub.HALAngularVelocityInPublisher()
        hal_magneticin_pub = halpub.HALMagneticInPublisher()
        hal_eulerin_pub = halpub.HALEulerInPublisher()
        hal_tempin_pub = halpub.HALTempInPublisher()

        # SensorHw messages
        hal_ultrasonicin_pub = halpub.HALSensorArrayInPublisher()
        hal_infraredin_pub = halpub.HALSensorArrayInPublisher()

        '''
        ----------------------------------------------------------------------
        Lookup table from message type to message and publisher
        ----------------------------------------------------------------------
        '''

        # PsocHw Message to Publisher mapping
        self._psochw_callback_dict = {
            hwmsg.MSG_ID_STATUS: hal_statusin_pub,
            hwmsg.MSG_ID_SPEED : hal_speedin_pub,
            hwmsg.MSG_ID_POSITION : hal_positionin_pub,
            hwmsg.MSG_ID_HEADING : hal_headingin_pub,
            hwmsg.MSG_ID_HEARTBEAT: hal_heartbeatin_pub
        }

        # ImuHw Message to Publisher mapping
        self._imuhw_callback_dict = {
            hwmsg.MSG_ID_ORIENTATION : hal_orientationin_pub,
            hwmsg.MSG_ID_LINEAR_ACCEL : hal_linearaccelin_pub,
            hwmsg.MSG_ID_ANGULAR_VELOCITY : hal_angularvelocityin_pub,
            hwmsg.MSG_ID_MAGNETIC : hal_magneticin_pub,
            hwmsg.MSG_ID_EULER : hal_eulerin_pub,
            hwmsg.MSG_ID_TEMPERATURE : hal_tempin_pub
        }

        # SensorHw Message to publisher mapping
        self._sensorhw_callback_dict = {
            hwmsg.MSG_ID_ULTRASONIC_FRONT : hal_ultrasonicin_pub,
            hwmsg.MSG_ID_ULTRASONIC_REAR : hal_ultrasonicin_pub,
            hwmsg.MSG_ID_INFRARED_FRONT : hal_infraredin_pub,
            hwmsg.MSG_ID_INFRARED_REAR: hal_infraredin_pub
        }

    def _psochw_driver_cb(self, psochw_msg):
        """
        Receives messages from the psochw driver and dispatches it to the appropriate publisher
        """
        rospy.logdebug("HALNode received psochw msg {}, {}".format(psochw_msg.type, psochw_msg.data))

        try:
            p = self._psochw_callback_dict[psochw_msg.type]
            p.publish(psochw_msg.data)
        except KeyError as err:
            rospy.logwarn("PsocHw Callback: Unknown msg {} - {}".format(psochw_msg.type, err))

    def _imuhw_callback(self, imuhw_msg):
        """
        Receives messages from the imuhw driver
        """
        rospy.logdebug("HALNode received imuhw msg {}".format(imuhw_msg))

        try:
            p = self._imuhw_callback_dict[imuhw_msg.type]
            p.publish(imuhw_msg.data)
        except KeyError as err:
            rospy.logwarn("ImuHw Callback: Unknown msg {} - {}".format(imuhw_msg.type, err))

    def _sensorhw_callback(self, sensorhw_msg):
        """
        Receives messages from the sensorhw driver
        """
        rospy.logdebug("HALNode recieved sensorhw msg {}".format(sensorhw_msg))

        ultrasonic_change = False
        infrared_change = False

        try:
            p = self._sensorhw_callback_dict[sensorhw_msg.type]

            if sensorhw_msg.type == hwmsg.MSG_ID_ULTRASONIC_FRONT:
                self._ultrasonic_data[SENSOR_DATA_FRONT] = sensorhw_msg.data
                ultrasonic_change = True
            elif sensorhw_msg.type == hwmsg.MSG_ID_ULTRASONIC_REAR:
                self._ultrasonic_rear[SENSOR_DATA_REAR] = sensorhw_msg.data
                ultrasonic_change = True
            elif sensorhw_msg.type == hwmsg.MSG_ID_INFRARED_FRONT:
                self._infrared_data[SENSOR_DATA_FRONT] = sensorhw_msg.data
                infrared_change = True
            elif sensorhw_msg.type == hwmsg.MSG_ID_INFRARED_REAR:
                self._infrared_data[SENSOR_DATA_REAR] = sensorhw_msg.data
                infrared_change = True

            if ultrasonic_change:
                p.publish(self._ultrasonic_data)

            if infrared_change:
                p.publish(self._infrared_data)

        except KeyError as err:
            rospy.logwarn("SensorHw Callback: Unknown msg {} - {}".format(sensorhw_msg.type, err))

    def _hal_controloutmsg_callback(self, msg):
        """
        Receives control data destined for the PsocHw driver
        """
        rospy.logdebug("HALControlOut: {}".format(msg))
        self._psochw.set_control(msg.device, msg.debug)

    def _hal_speedoutmsg_callback(self, msg):
        """
        Receives speed data destined for the PsocHw driver
        """
        #rospy.loginfo("HALSpeedOut: {}".format(str(msg)))
        self._psochw.set_speed(msg.linear, msg.angular)

    def start(self):
        """
        Performs operations that must happen before the main loop starts but after __init__
        """
        rospy.loginfo("HALNode starting ...")
        self._psochw and self._psochw.start()
        self._imuhw and self._imuhw.start()
        self._sensorhw and self._sensorhw.start()
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
        if self._psochw:
            rospy.loginfo("PsocHw shutting down ...")
            self._psochw.shutdown()
            rospy.loginfo("PsocHw shutdown")
            del self._psochw
        if self._imuhw:
            rospy.loginfo("ImuHw shutting down ...")
            self._imuhw.shutdown()
            rospy.loginfo("ImuHw shutdown")
            del self._imuhw
        if self._sensorhw:
            rospy.loginfo("SensorHw shutting down ...")
            self._sensorhw.shutdown()
            rospy.loginfo("SensorHw shutdown")
            del self._sensorhw


if __name__ == "__main__":
    try:
        halnode = HALNode(debug=False)
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
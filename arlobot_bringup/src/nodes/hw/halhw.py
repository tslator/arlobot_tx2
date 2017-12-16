#!/usr/bin/env python
from __future__ import print_function
"""
---------------------------------------------------------------------------------------------------
File: halhw.py

Description: Provides implementation of the hardware which is accessed by the HAL node.

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
from math import cos, sin

# Third-Party
import rospy

# Project
from psochwfactory import PsocHwFactory, PsocHwFactoryError
from imuhwfactory import ImuHwFactory, ImuHwFactoryError
from sensorhwfactory import SensorHwFactory, SensorHwFactoryError
from sensorhw import NUM_SENSORS, SENSOR_DATA_FRONT, SENSOR_DATA_REAR
import messages as hwmsg
from arlobot_bringup.msg import (
    # PsocHw Messages
    HALStatusIn, HALSpeedIn, HALPositionIn, HALHeadingIn, HALHeartbeatIn, HALSpeedOut, HALControlOut,
    # ImuHw Messages
    HALOrientationIn, HALLinearAccelIn, HALAngularVelocityIn, HALMagneticIn, HALEulerIn, HALTempIn,
    # SensorHw Messages
    HALSensorArrayIn
)

#from messages import Message, SpeedData, PositionData, HeadingData, MSG_ID_SPEED, MSG_ID_POSITION, MSG_ID_HEADING
import pubs.halpub as halpub
from common import Logger

"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

class ArlobotHardware(object):
    def __init__(self):

        self._ultrasonic_data = [0.0,] * NUM_SENSORS
        self._infrared_data = [0.0,] * NUM_SENSORS

        self._psoc_if_dict = {
            'i2c': PsocHwFactory.create_i2c(driver_cb=self._psochw_driver_cb, logger=Logger('rospy')),
            'can': PsocHwFactory.create_can(driver_cb=self._psochw_driver_cb, logger=Logger('rospy')),
            'mock': PsocHwFactory.create_mock(driver_cb=self._psochw_driver_cb, logger=Logger('rospy'))
        }

        psoc_interface = rospy.get_param('Psoc Interface', 'i2c')
        enable_imu = rospy.get_param('Enable IMU Hw', True)
        enable_sensor = rospy.get_param('Enable Sensor Hw', True)

        self._imuhw = None
        self._sensorhw = None

        try:
            try:
                self._psochw = self._psoc_if_dict[psoc_interface]
            except PsocHwFactoryError as err:
                raise HALNodeError("Failure to instantiate PSOC hardware", err)
        except KeyError as err:
            raise HALNodeError("Unknown PSOC interface {}".format(psoc_interface), err)

        if enable_imu:
            try:
                self._imuhw = ImuHwFactory.create_i2c(driver_cb=self._imuhw_callback, logger=Logger('rospy'))
            except ImuHwFactoryError as err:
                raise HALNodeError("Failure to instantiate IMU hardware", err)

        if enable_sensor:
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
        self._psochw and self._psochw.set_control(msg.device, msg.debug)

    def _hal_speedoutmsg_callback(self, msg):
        """
        Receives speed data destined for the PsocHw driver
        """
        #rospy.loginfo("HALSpeedOut: {}".format(str(msg)))
        self._psochw and self._psochw.set_speed(msg.linear, msg.angular)

    def start(self):
        rospy.loginfo("Starting PsocHw ...")
        self._psochw and self._psochw.start()

        rospy.loginfo("Starting ImuHw ...")
        self._imuhw and self._imuhw.start()

        rospy.loginfo("Starting SensorHw ...")
        self._sensorhw and self._sensorhw.start()

    def shutdown(self):
        rospy.loginfo("PsocHw shutting down ...")
        self._psochw and self._psochw.shutdown()
        rospy.loginfo("PsocHw shut down")

        rospy.loginfo("ImuHw shutting down ...")
        self._imuhw and self._imuhw.shutdown()
        rospy.loginfo("ImuHw shutdown")

        rospy.loginfo("SensorHw shutting down ...")
        self._sensorhw and self._sensorhw.shutdown()
        rospy.loginfo("SensorHw shutdown")



class SimulatedHardware(object):
    def __init__(self):
        pass

        self.x = 0.0
        self.y = 0.0
        self.v = 0.0
        self.w = 0.0
        self.th = 0.0
        self.then = 0.0

        '''
        ----------------------------------------------------------------------
        Subscribers for messages from other nodes to the HAL ('Out' messages)
        ----------------------------------------------------------------------
        '''
        self._hal_speedout_sub = rospy.Subscriber('HALSpeedOut', 
                                                  HALSpeedOut, 
                                                  self._hal_speedoutmsg_callback,
                                                  queue_size=1)

        '''
        ----------------------------------------------------------------------
        Publishers for messages from the HAL to other nodes ('In' messages)
        ----------------------------------------------------------------------
        '''

        # PsocHw messages
        self._hal_speedin_pub = halpub.HALSpeedInPublisher()
        self._hal_positionin_pub = halpub.HALPositionInPublisher()
        self._hal_headingin_pub = halpub.HALHeadingInPublisher()


    def _hal_speedoutmsg_callback(self, msg):
        elapsed = rospy.Time.now() - self.then
        elapsed = elapsed.to_sec()

        self.then = rospy.Time.now()

        self.v = msg.linear
        self.w = msg.angular

        # A couple of questions:
        #  Why compute x and y as they are not used?
        #  Why is meas_y not -sin?
        #x = cos(self.meas_th) * self.v * elapsed
        #y = -sin(self.meas_th) * self.v * elapsed
        self.x += cos(self.th) * self.v * elapsed
        self.y += sin(self.th) * self.v * elapsed
        self.th += self.w * elapsed

        self._hal_speedin_pub.publish(hwmsg.SpeedData(linear=self.v, angular=self.w))
        self._hal_positionin_pub.publish(hwmsg.PositionData(x=self.x, y=self.y))
        self._hal_headingin_pub.publish(hwmsg.HeadingData(self.th))    

    def start(self):
        # Note: Cannot use Time until the node is initialized
        self.then = rospy.Time.now()

    def shutdown(self):
        pass

# --- EOF ---

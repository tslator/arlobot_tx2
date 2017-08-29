#!/usr/bin/env python
"""
---------------------------------------------------------------------------------------------------
File: messages.py

Description: Provides definitions of messages that are pass between drivers and the *hw modules.

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
from collections import namedtuple

"""
---------------------------------------------------------------------------------------------------
Message Ids
---------------------------------------------------------------------------------------------------
"""

# Psoc Hw Messages
MSG_ID_CONTROL = 1
MSG_ID_DEBUG = 2
MSG_ID_STATUS = 3
MSG_ID_SPEED = 4
MSG_ID_POSITION = 5
MSG_ID_HEADING = 6
MSG_ID_HEARTBEAT = 7

# IMU Hw Messages
MSG_ID_ORIENTATION = 20
MSG_ID_LINEAR_ACCEL = 21
MSG_ID_ANGULAR_VELOCITY = 22
MSG_ID_GYROSCOPE = 23
MSG_ID_MAGNETIC = 24
MSG_ID_EULER = 25
MSG_ID_TEMPERATURE = 26

MSG_ID_ULTRASONIC_FRONT = 30
MSG_ID_ULTRASONIC_REAR = 31
MSG_ID_INFRARED_FRONT = 32
MSG_ID_INFRARED_REAR = 33

# Mock Message
MSG_ID_MOCK = 1000

"""
---------------------------------------------------------------------------------------------------
Message Types

Defines a generic message with fields type and data where type is a message id and data is the
specific data associated with a message id.
---------------------------------------------------------------------------------------------------
"""

Message = namedtuple('Message', 'type data')


# PsocHw data definitions

# StatusData - contains device and calibration information
StatusData = namedtuple('StatusData', 'device calibration')
# ControlData - contains device control data
ControlData = namedtuple('ControlData', 'device debug')
# DebugData - contains device debug data
DebugData = namedtuple('DebugData', 'debug')
# SpeedData - contains the left, right speed of the motors (meter/second)
SpeedData = namedtuple('SpeedData', 'linear angular')
# DistanceData - contains the left, right distance travelled (meter)
PositionData = namedtuple('PositionData', 'x y')
# HeadingData - contains the heading calculation (not from IMU) (radians)
HeadingData = namedtuple('HeadingData', 'heading')
# HeartbeatData - contains the heartbeat count value (seconds)
HeartbeatData = namedtuple('HeartbeatData', 'heartbeat')


# ImuHwData

# OrientationData - 
OrientationData = namedtuple('OrientationData', 'x y z w')
# LinearAccelData - 
LinearAccelData = namedtuple('LinearAccelData', 'x y z')
# AngularVelocityData (gyroscope)
AngularVelocityData = namedtuple('AngularVelocityData', 'x y z')
# MagneticData - 
MagneticData = namedtuple('MagneticData', 'x y z')
# EulerData - 
EulerData = namedtuple('EulerData', 'yaw roll pitch')
# TemperatureData -
TemperatureData = namedtuple('TemperatureData', 'f c')

# SensorHwData
SensorData = namedtuple('SensorData', 'source data')


# Mock Data

MockData = namedtuple('MockData', 'data')

#--- EOF ---



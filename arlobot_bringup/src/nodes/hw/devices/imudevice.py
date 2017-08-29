#! /usr/bin/env python
"""
The imudevice module defines class to create an IMU Device
"""
from __future__ import print_function

# Standard Library
import time

# 3rd Party
# None

# Project
from .i2cdevice import I2CDeviceError
from .mockdevice import MockDevice
from .basedevice import BaseDevice
from .bno055 import BNO055
from ..messages import Message, MSG_ID_EULER, EulerData, MSG_ID_MAGNETIC, MagneticData, MSG_ID_LINEAR_ACCEL, \
    LinearAccelData, MSG_ID_ANGULAR_VELOCITY, AngularVelocityData, MSG_ID_ORIENTATION, OrientationData, \
    MSG_ID_TEMPERATURE, TemperatureData
from utils.logger import Logger


class ImuDeviceError(Exception):
    pass


class ImuDevice(BaseDevice):

    def __init__(self, name, device=None, timeout=5, logger=None):
        """
        Initialize the ImuDevice
        :param timeout: the amount of time to wait for calibration status to settle
        :return: None
        """
        super(ImuDevice, self).__init__(name)

        self._logger = logger or Logger()
        self._device = device or MockDevice('mock device')
        self._msg_gen = None

        try:
            self._bno = BNO055(i2c=self._device)
        except I2CDeviceError as err:
            raise ImuDeviceError("Unable to instantiate BNO055", err)

        self._start_hardware(timeout)

        # Mapping between message ids and functions in the BNO055 instance
        # The lambda is need to ensure the Message is dynamically created and data is reacquired each cycle.
        self._msgs = [
            lambda: Message(type=MSG_ID_EULER, data=EulerData(*self._bno.read_euler())),
            lambda: Message(type=MSG_ID_MAGNETIC, data=MagneticData(*self._bno.read_magnetometer())),
            lambda: Message(type=MSG_ID_LINEAR_ACCEL, data=LinearAccelData(*self._bno.read_linear_acceleration())),
            lambda: Message(type=MSG_ID_ANGULAR_VELOCITY, data=AngularVelocityData(*self._bno.read_gyroscope())),
            lambda: Message(type=MSG_ID_ORIENTATION, data=OrientationData(*self._bno.read_quaternion())),
            lambda: Message(type=MSG_ID_TEMPERATURE, data=TemperatureData(*self._read_temp_wrapper()))
        ]

        self._msg_cycle = None

    def _read_temp_wrapper(self):
        c = self._bno.read_temp()
        f = (c * 9.0)/5.0 + 32

        return f, c

    def _start_hardware(self, timeout):

        self._bno.begin()

        status, self_test, error = self._bno.get_system_status()
        self._bno.set_calibration()
        systm, _, _, _ = self._bno.get_calibration_status()
        timed_out = False
        timeout = time.time() + timeout
        while systm != 3 and not timed_out:
            systm, gyro, accel, mag = self._bno.get_calibration_status()
            #self._logger.debug('sys: {}, gyro: {}, accel: {}, mag: {}'.format(systm, gyro, accel, mag))
            time.sleep(0.1)
            timed_out = timeout - time.time() <= 0
        else:
            if systm != 3:
                self._logger.fatal("BNO055 - system status is invalid: {}".format(bin(status)))
                return

            if timed_out:
                self._logger.fatal("BNO055 - timeout waiting for system status to be valid")
                return

        systm, gyro, accel, mag = self._bno.get_calibration_status()
        fmt_str = 'BNO055 - status: {}, self_test: {}, error: {}'.format(bin(status), bin(self_test), bin(error))
        self._logger.debug(fmt_str)
        fmt_str = 'sys: {}, gyro: {}, accel: {}, mag: {}'.format(systm, gyro, accel, mag)
        self._logger.info(fmt_str)

    # Note: The following two routines together implement a renewable generator.  Upon the initial call to recv, the
    # message cycle will be None and require resetting, i.e., a new generator.  With a generator in place, recv yields
    # each message in the generator until the end of the cycle is reached.  Then the process is repeated.
    #
    # The generator consists of a sequence of functions that populate a Message with data from each type of data
    # supported (see self._msgs)
    def _reset(self):
        if self._msg_cycle is None:
            self._msg_cycle = (self._msgs[i]() for i in range(len(self._msgs)))

    def recv(self):
        self._reset()

        try:
            msg = next(self._msg_cycle)
        except StopIteration:
            self._msg_cycle = None
            self._reset()
            msg = next(self._msg_cycle)

        return msg

    def send(self):
        """
        There is no need to pass data to the Imu device
        :return:
        """
        pass


def module_test():
    import logging
    from i2cdevice import I2CDevice

    logger = Logger()

    logger.debug("Start Module Test")

    i2c_device = I2CDevice(I2CDevice.DEV_I2C_1, logger=logger)
    imu = ImuDevice(name="imu device", device=i2c_device, timeout=5, logger=logger)

    logger.debug("End Module Test")


if __name__ == "__main__":
    module_test()

# --- EOF ---


# Adafruit BNO055 Absolute Orientation Sensor Library
# Copyright (c) 2015 Adafruit Industries
# Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
from __future__ import print_function

# Standard Library
import time
import os

# 3rd Party
# None

# Project
from i2cdevice import I2CDeviceError
from common import Logger


# I2C addresses
BNO055_ADDRESS_A = 0x28
BNO055_ADDRESS_B = 0x29
BNO055_ID = 0xA0

# Page id register definition
BNO055_PAGE_ID_ADDR = 0X07

# PAGE0 REGISTER DEFINITION START
BNO055_CHIP_ID_ADDR = 0x00
BNO055_ACCEL_REV_ID_ADDR = 0x01
BNO055_MAG_REV_ID_ADDR = 0x02
BNO055_GYRO_REV_ID_ADDR = 0x03
BNO055_SW_REV_ID_LSB_ADDR = 0x04
BNO055_SW_REV_ID_MSB_ADDR = 0x05
BNO055_BL_REV_ID_ADDR = 0X06

# Accel data register
BNO055_ACCEL_DATA_X_LSB_ADDR = 0X08
BNO055_ACCEL_DATA_X_MSB_ADDR = 0X09
BNO055_ACCEL_DATA_Y_LSB_ADDR = 0X0A
BNO055_ACCEL_DATA_Y_MSB_ADDR = 0X0B
BNO055_ACCEL_DATA_Z_LSB_ADDR = 0X0C
BNO055_ACCEL_DATA_Z_MSB_ADDR = 0X0D

# Mag data register
BNO055_MAG_DATA_X_LSB_ADDR = 0X0E
BNO055_MAG_DATA_X_MSB_ADDR = 0X0F
BNO055_MAG_DATA_Y_LSB_ADDR = 0X10
BNO055_MAG_DATA_Y_MSB_ADDR = 0X11
BNO055_MAG_DATA_Z_LSB_ADDR = 0X12
BNO055_MAG_DATA_Z_MSB_ADDR = 0X13

# Gyro data registers
BNO055_GYRO_DATA_X_LSB_ADDR = 0X14
BNO055_GYRO_DATA_X_MSB_ADDR = 0X15
BNO055_GYRO_DATA_Y_LSB_ADDR = 0X16
BNO055_GYRO_DATA_Y_MSB_ADDR = 0X17
BNO055_GYRO_DATA_Z_LSB_ADDR = 0X18
BNO055_GYRO_DATA_Z_MSB_ADDR = 0X19

# Euler data registers
BNO055_EULER_H_LSB_ADDR = 0X1A
BNO055_EULER_H_MSB_ADDR = 0X1B
BNO055_EULER_R_LSB_ADDR = 0X1C
BNO055_EULER_R_MSB_ADDR = 0X1D
BNO055_EULER_P_LSB_ADDR = 0X1E
BNO055_EULER_P_MSB_ADDR = 0X1F

# Quaternion data registers
BNO055_QUATERNION_DATA_W_LSB_ADDR = 0X20
BNO055_QUATERNION_DATA_W_MSB_ADDR = 0X21
BNO055_QUATERNION_DATA_X_LSB_ADDR = 0X22
BNO055_QUATERNION_DATA_X_MSB_ADDR = 0X23
BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0X24
BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0X25
BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0X26
BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0X27

# Linear acceleration data registers
BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0X28
BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR = 0X29
BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0X2A
BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0X2B
BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0X2C
BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0X2D

# Gravity data registers
BNO055_GRAVITY_DATA_X_LSB_ADDR = 0X2E
BNO055_GRAVITY_DATA_X_MSB_ADDR = 0X2F
BNO055_GRAVITY_DATA_Y_LSB_ADDR = 0X30
BNO055_GRAVITY_DATA_Y_MSB_ADDR = 0X31
BNO055_GRAVITY_DATA_Z_LSB_ADDR = 0X32
BNO055_GRAVITY_DATA_Z_MSB_ADDR = 0X33

# Temperature data register
BNO055_TEMP_ADDR = 0X34

# Status registers
BNO055_CALIB_STAT_ADDR = 0X35
BNO055_SELFTEST_RESULT_ADDR = 0X36
BNO055_INTR_STAT_ADDR = 0X37

BNO055_SYS_CLK_STAT_ADDR = 0X38
BNO055_SYS_STAT_ADDR = 0X39
BNO055_SYS_ERR_ADDR = 0X3A

# Unit selection register
BNO055_UNIT_SEL_ADDR = 0X3B
BNO055_DATA_SELECT_ADDR = 0X3C

# Mode registers
BNO055_OPR_MODE_ADDR = 0X3D
BNO055_PWR_MODE_ADDR = 0X3E

BNO055_SYS_TRIGGER_ADDR = 0X3F
BNO055_TEMP_SOURCE_ADDR = 0X40

# Axis remap registers
BNO055_AXIS_MAP_CONFIG_ADDR = 0X41
BNO055_AXIS_MAP_SIGN_ADDR = 0X42

# Axis remap values
AXIS_REMAP_X = 0x00
AXIS_REMAP_Y = 0x01
AXIS_REMAP_Z = 0x02
AXIS_REMAP_POSITIVE = 0x00
AXIS_REMAP_NEGATIVE = 0x01

# SIC registers
BNO055_SIC_MATRIX_0_LSB_ADDR = 0X43
BNO055_SIC_MATRIX_0_MSB_ADDR = 0X44
BNO055_SIC_MATRIX_1_LSB_ADDR = 0X45
BNO055_SIC_MATRIX_1_MSB_ADDR = 0X46
BNO055_SIC_MATRIX_2_LSB_ADDR = 0X47
BNO055_SIC_MATRIX_2_MSB_ADDR = 0X48
BNO055_SIC_MATRIX_3_LSB_ADDR = 0X49
BNO055_SIC_MATRIX_3_MSB_ADDR = 0X4A
BNO055_SIC_MATRIX_4_LSB_ADDR = 0X4B
BNO055_SIC_MATRIX_4_MSB_ADDR = 0X4C
BNO055_SIC_MATRIX_5_LSB_ADDR = 0X4D
BNO055_SIC_MATRIX_5_MSB_ADDR = 0X4E
BNO055_SIC_MATRIX_6_LSB_ADDR = 0X4F
BNO055_SIC_MATRIX_6_MSB_ADDR = 0X50
BNO055_SIC_MATRIX_7_LSB_ADDR = 0X51
BNO055_SIC_MATRIX_7_MSB_ADDR = 0X52
BNO055_SIC_MATRIX_8_LSB_ADDR = 0X53
BNO055_SIC_MATRIX_8_MSB_ADDR = 0X54

# Accelerometer Offset registers
ACCEL_OFFSET_X_LSB_ADDR = 0X55
ACCEL_OFFSET_X_MSB_ADDR = 0X56
ACCEL_OFFSET_Y_LSB_ADDR = 0X57
ACCEL_OFFSET_Y_MSB_ADDR = 0X58
ACCEL_OFFSET_Z_LSB_ADDR = 0X59
ACCEL_OFFSET_Z_MSB_ADDR = 0X5A

# Magnetometer Offset registers
MAG_OFFSET_X_LSB_ADDR = 0X5B
MAG_OFFSET_X_MSB_ADDR = 0X5C
MAG_OFFSET_Y_LSB_ADDR = 0X5D
MAG_OFFSET_Y_MSB_ADDR = 0X5E
MAG_OFFSET_Z_LSB_ADDR = 0X5F
MAG_OFFSET_Z_MSB_ADDR = 0X60

# Gyroscope Offset register s
GYRO_OFFSET_X_LSB_ADDR = 0X61
GYRO_OFFSET_X_MSB_ADDR = 0X62
GYRO_OFFSET_Y_LSB_ADDR = 0X63
GYRO_OFFSET_Y_MSB_ADDR = 0X64
GYRO_OFFSET_Z_LSB_ADDR = 0X65
GYRO_OFFSET_Z_MSB_ADDR = 0X66

# Radius registers
ACCEL_RADIUS_LSB_ADDR = 0X67
ACCEL_RADIUS_MSB_ADDR = 0X68
MAG_RADIUS_LSB_ADDR = 0X69
MAG_RADIUS_MSB_ADDR = 0X6A

# Power modes
POWER_MODE_NORMAL = 0X00
POWER_MODE_LOWPOWER = 0X01
POWER_MODE_SUSPEND = 0X02

# Operation mode settings
OPERATION_MODE_CONFIG = 0X00
OPERATION_MODE_ACCONLY = 0X01
OPERATION_MODE_MAGONLY = 0X02
OPERATION_MODE_GYRONLY = 0X03
OPERATION_MODE_ACCMAG = 0X04
OPERATION_MODE_ACCGYRO = 0X05
OPERATION_MODE_MAGGYRO = 0X06
OPERATION_MODE_AMG = 0X07
OPERATION_MODE_IMUPLUS = 0X08
OPERATION_MODE_COMPASS = 0X09
OPERATION_MODE_M4G = 0X0A
OPERATION_MODE_NDOF_FMC_OFF = 0X0B
OPERATION_MODE_NDOF = 0X0C

#-------------------------------------------------------
# Read in the calibration data when the module is loaded
#-------------------------------------------------------

CALIBRATION_DATA = [242, 255, 253, 255, 10, 0, 75, 254, 132, 0, 212, 254, 0, 0, 254, 255, 0, 0, 232, 3, 90, 3]


def load_calibration():
    global CALIBRATION_DATA
    logger = Logger()

    try:
        # To load the calibration data it is necessary to get the absolutate path.  The calibration data is stored in the
        # same directory as this module.  The __file__ attribute contains the needed path information.
        abs_path = os.path.dirname(os.path.abspath(__file__))
        f = open(os.path.join(abs_path, 'bno055.cal'), 'r')
    except IOError as err:
        logger.fatal("Unable to load calibration data from bno055.cal - {}".format(err))
    else:
        with f:
            l = f.readline()
            s = l[1:-1]
            try:
                CALIBRATION_DATA = [int(i) for i in s.split(', ')]
                logger.debug("Calibration Data Loaded: {}".format(CALIBRATION_DATA))
            except ValueError as err:
                logger.fatal("Invalid calibration data: {} - {}".format(s, err))

# Load calibration when the module is imported
load_calibration()


class BNO055(object):
    """
    Create a class for BNO055 hardware
    """
    def __init__(self, address=0x28, i2c=None, rst=None, gpio=None, logger=None):
        """
        Initialize the BNO055 class
        :param address: the I2C address
        :param i2c: the i2c device
        :param rst: a reset pin
        :param gpio: a gpio package
        :return: None
        """
        # If reset pin is provided save it and a reference to provided GPIO
        # bus (or the default system GPIO bus if none is provided).
        self._rst = rst
        if self._rst is not None:
            if gpio is None:
                import Adafruit_GPIO as GPIO
                gpio = GPIO.get_platform_gpio()
            self._gpio = gpio
            # Setup the reset pin as an output at a high level.
            self._gpio.setup(self._rst, GPIO.OUT)
            self._gpio.set_high(self._rst)
            # Wait a 650 milliseconds in case setting the reset high reset the chip.
            time.sleep(0.65)
            
        self._address = address
        self._i2c_device = i2c
        self._mode = OPERATION_MODE_NDOF
        self._logger = logger or Logger()

    def _reset(self):
        """
        Reset the BNO055 hardware
        :return: None
        """
        # Reset the device.
        if self._rst is not None:
            # Use the hardware reset pin if provided.
            # Go low for a short period, then high to signal a reset.
            self._gpio.set_low(self._rst)
            time.sleep(0.01)  # 10ms
            self._gpio.set_high(self._rst)
        else:
            # Else use the reset command.  Note that ack=False is sent because
            # the chip doesn't seem to ack a reset in serial mode (by design?).
            self._i2c_device.write_uint8(self._address, BNO055_SYS_TRIGGER_ADDR, 0x20)
        # Wait 650ms after reset for chip to be ready (as suggested
        # in datasheet).
        time.sleep(0.65)
        
    def _config_mode(self):
        """
        Place the BNO055 hardware into configuration mode
        :return: None
        """
        # Enter configuration mode.
        self.set_mode(OPERATION_MODE_CONFIG)

    def _operation_mode(self):
        """
        Please the BNO055 hardware into operation mode
        :return: None
        """
        # Enter operation mode to read sensor data.
        self.set_mode(self._mode)

    def begin(self, mode=OPERATION_MODE_NDOF):
        """
        Startup the BNO055 hardware in the specified mode
        :param mode: the specified mode (default: operational)
        :return: True if successful; otherwise, False
        """
        """Initialize the BNO055 sensor.  Must be called once before any other
        BNO055 library functions.  Will return True if the BNO055 was
        successfully initialized, and False otherwise.
        """
        # Save the desired normal operation mode.
        self._mode = mode
        # First send a thow-away command and ignore any response or I2C errors
        # just to make sure the BNO is in a good state and ready to accept
        # commands (this seems to be necessary after a hard power down).
        try:
            self._i2c_device.write_uint8(self._address, BNO055_PAGE_ID_ADDR, 0)
        except I2CDeviceError as err:
            # Swallow an IOError that might be raised by an I2C issue.  Only do
            # this for this very first command to help get the BNO and board's
            # I2C into a clear state ready to accept the next commands.
            self._logger.warn('Initial write error: {}'.format(err))
            
        # Make sure we're in config mode and on page 0.
        self._config_mode()
        self._i2c_device.write_uint8(self._address, BNO055_PAGE_ID_ADDR, 0)
        # Check the chip ID
        bno_id = self._i2c_device.read_uint8(self._address, BNO055_CHIP_ID_ADDR)

        self._logger.debug('Read chip ID: 0x{0:02X}'.format(bno_id))
        if bno_id != BNO055_ID:
            return False
            
        self._reset()
        
        # Set to normal power mode.
        self._i2c_device.write_uint8(self._address, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL)
        # Default to internal oscillator.
        self._i2c_device.write_uint8(self._address, BNO055_SYS_TRIGGER_ADDR, 0x00)
        # Enter normal operation mode.
        self._operation_mode()

        return True

    def set_mode(self, mode):
        """
        Set BNO055 hardware mode
        :param mode: the specified mode
        :return: None
        """
        """Set operation mode for BNO055 sensor.  Mode should be a value from
        table 3-3 and 3-5 of the datasheet:
          http://www.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
        """
        self._i2c_device.write_uint8(self._address, BNO055_OPR_MODE_ADDR, mode & 0xFF)
        # Delay for 30 milliseconds (datsheet recommends 19ms, but a little more
        # can't hurt and the kernel is going to spend some unknown amount of time
        # too).
        time.sleep(0.03)

    def get_revision(self):
        """
        Get the BNO055 hardware revision data
        :return: revision data
        """
        """Return a tuple with revision information about the BNO055 chip.  Will
        return 5 values:
          - Software revision
          - Bootloader version
          - Accelerometer ID
          - Magnetometer ID
          - Gyro ID
        """
        # Read revision values.
        accel = self._i2c_device.read_uint8(self._address, BNO055_ACCEL_REV_ID_ADDR)
        mag = self._i2c_device.read_uint8(self._address, BNO055_MAG_REV_ID_ADDR)
        gyro = self._i2c_device.read_uint8(self._address, BNO055_GYRO_REV_ID_ADDR)
        bl = self._i2c_device.read_uint8(self._address, BNO055_BL_REV_ID_ADDR)
        sw_lsb = self._i2c_device.read_uint8(self._address, BNO055_SW_REV_ID_LSB_ADDR)
        sw_msb = self._i2c_device.read_uint8(self._address, BNO055_SW_REV_ID_MSB_ADDR)
        sw = ((sw_msb << 8) | sw_lsb) & 0xFFFF
        # Return the results as a tuple of all 5 values.
        return (sw, bl, accel, mag, gyro)

    def set_external_crystal(self, external_crystal):
        """
        Enable external crystal
        :param external_crystal: True to enable; otherwise, False  
        :return: None
        """
        """Set if an external crystal is being used by passing True, otherwise
        use the internal oscillator by passing False (the default behavior).
        """
        # Switch to configuration mode.
        self._config_mode()
        # Set the clock bit appropriately in the SYS_TRIGGER register.
        if external_crystal:
            self._i2c_device.write_uint8(self._address, BNO055_SYS_TRIGGER_ADDR, 0x80)
        else:
            self._i2c_device.write_uint8(self._address, BNO055_SYS_TRIGGER_ADDR, 0x00)
        # Go back to normal operation mode.
        self._operation_mode()

    def get_system_status(self, run_self_test=True):
        """
        Get system status
        :param run_self_test: if True, self test will be run 
        :return: system status, test result, and any error
        """
        """Return a tuple with status information.  Three values will be returned:
          - System status register value with the following meaning:
              0 = Idle
              1 = System Error
              2 = Initializing Peripherals
              3 = System Initialization
              4 = Executing Self-Test
              5 = Sensor fusion algorithm running
              6 = System running without fusion algorithms
          - Self test result register value with the following meaning:
              Bit value: 1 = test passed, 0 = test failed
              Bit 0 = Accelerometer self test
              Bit 1 = Magnetometer self test
              Bit 2 = Gyroscope self test
              Bit 3 = MCU self test
              Value of 0x0F = all good!
          - System error register value with the following meaning:
              0 = No error
              1 = Peripheral initialization error
              2 = System initialization error
              3 = Self test result failed
              4 = Register map value out of range
              5 = Register map address out of range
              6 = Register map write error
              7 = BNO low power mode not available for selected operation mode
              8 = Accelerometer power mode not available
              9 = Fusion algorithm configuration error
             10 = Sensor configuration error
        If run_self_test is passed in as False then no self test is performed and
        None will be returned for the self test result.  Note that running a
        self test requires going into config mode which will stop the fusion
        engine from running.
        """
        self_test = None
        if run_self_test:
            # Switch to configuration mode if running self test.
            self._config_mode()
            # Perform a self test.
            sys_trigger = self._i2c_device.read_uint8(self._address, BNO055_SYS_TRIGGER_ADDR)
            self._i2c_device.write_uint8(self._address, BNO055_SYS_TRIGGER_ADDR, sys_trigger | 0x1)
            # Wait for self test to finish.
            time.sleep(1.0)
            # Read test result.
            self_test = self._i2c_device.read_uint8(self._address, BNO055_SELFTEST_RESULT_ADDR)
            # Go back to operation mode.
            self._operation_mode()
        # Now read status and error registers.
        status = self._i2c_device.read_uint8(self._address, BNO055_SYS_STAT_ADDR)
        error = self._i2c_device.read_uint8(self._address, BNO055_SYS_ERR_ADDR)
        # Return the results as a tuple of all 3 values.
        return (status, self_test, error)

    def get_calibration_status(self):
        """
        Get calibration status
        :return: system, gyroscope, accelerometer and magnetometer status
        """
        """Read the calibration status of the sensors and return a 4 tuple with
        calibration status as follows:
          - System, 3=fully calibrated, 0=not calibrated
          - Gyroscope, 3=fully calibrated, 0=not calibrated
          - Accelerometer, 3=fully calibrated, 0=not calibrated
          - Magnetometer, 3=fully calibrated, 0=not calibrated
        """
        # Return the calibration status register value.
        cal_status = self._i2c_device.read_uint8(self._address, BNO055_CALIB_STAT_ADDR)
        sys = (cal_status >> 6) & 0x03
        gyro = (cal_status >> 4) & 0x03
        accel = (cal_status >> 2) & 0x03
        mag = cal_status & 0x03
        # Return the results as a tuple of all 3 values.
        return (sys, gyro, accel, mag)

    def get_calibration(self):
        """
        Get calibration data
        :return: calibration data
        """
        """Return the sensor's calibration data and return it as an array of
        22 bytes. Can be saved and then reloaded with the set_calibration function
        to quickly calibrate from a previously calculated set of calibration data.
        """
        # Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
        self._config_mode()
        # Read the 22 bytes of calibration data and convert it to a list (from
        # a bytearray) so it's more easily serialized should the caller want to
        # store it.
        cal_data = self._i2c_device.read_array(self._address, ACCEL_OFFSET_X_LSB_ADDR, 22, 'B')
        # Go back to normal operation mode.
        self._operation_mode()
        return cal_data

    def set_calibration(self, data=CALIBRATION_DATA):
        """
        Set calibration data
        :param data: calibration data
        :return: None
        """
        """Set the sensor's calibration data using a list of 22 bytes that
        represent the sensor offsets and calibration data.  This data should be
        a value that was previously retrieved with get_calibration (and then
        perhaps persisted to disk or other location until needed again).
        """
        # Check that 22 bytes were passed in with calibration data.
        if data is None or len(data) != 22:
            raise ValueError('Expected a list of 22 bytes for calibration data.')
        # Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
        self._config_mode()
        # Set the 22 bytes of calibration data.
        self._i2c_device.write_array(self._address, ACCEL_OFFSET_X_LSB_ADDR, data, 'B')
        # Go back to normal operation mode.
        self._operation_mode()

    def get_axis_remap(self):
        """Return a tuple with the axis remap register values.  This will return
        6 values with the following meaning:
          - X axis remap (a value of AXIS_REMAP_X, AXIS_REMAP_Y, or AXIS_REMAP_Z.
                          which indicates that the physical X axis of the chip
                          is remapped to a different axis)
          - Y axis remap (see above)
          - Z axis remap (see above)
          - X axis sign (a value of AXIS_REMAP_POSITIVE or AXIS_REMAP_NEGATIVE
                         which indicates if the X axis values should be positive/
                         normal or negative/inverted.  The default is positive.)
          - Y axis sign (see above)
          - Z axis sign (see above)
        Note that by default the axis orientation of the BNO chip looks like
        the following (taken from section 3.4, page 24 of the datasheet).  Notice
        the dot in the corner that corresponds to the dot on the BNO chip:
                           | Z axis
                           |
                           |   / X axis
                       ____|__/____
          Y axis     / *   | /    /|
          _________ /______|/    //
                   /___________ //
                  |____________|/
        """
        # Get the axis remap register value.
        map_config = self._i2c_device.read_uint8(self._address, BNO055_AXIS_MAP_CONFIG_ADDR)
        z = (map_config >> 4) & 0x03
        y = (map_config >> 2) & 0x03
        x = map_config & 0x03
        # Get the axis remap sign register value.
        sign_config = self._i2c_device.read_uint8(self._address, BNO055_AXIS_MAP_SIGN_ADDR)
        x_sign = (sign_config >> 2) & 0x01
        y_sign = (sign_config >> 1) & 0x01
        z_sign = sign_config & 0x01
        # Return the results as a tuple of all 3 values.
        return (x, y, z, x_sign, y_sign, z_sign)

    def set_axis_remap(self, x, y, z,
                       x_sign=AXIS_REMAP_POSITIVE, y_sign=AXIS_REMAP_POSITIVE,
                       z_sign=AXIS_REMAP_POSITIVE):
        """Set axis remap for each axis.  The x, y, z parameter values should
        be set to one of AXIS_REMAP_X, AXIS_REMAP_Y, or AXIS_REMAP_Z and will
        change the BNO's axis to represent another axis.  Note that two axises
        cannot be mapped to the same axis, so the x, y, z params should be a
        unique combination of AXIS_REMAP_X, AXIS_REMAP_Y, AXIS_REMAP_Z values.
        The x_sign, y_sign, z_sign values represent if the axis should be positive
        or negative (inverted).
        See the get_axis_remap documentation for information on the orientation
        of the axises on the chip, and consult section 3.4 of the datasheet.
        """
        # Switch to configuration mode.
        self._config_mode()
        # Set the axis remap register value.
        map_config = 0x00
        map_config |= (z & 0x03) << 4
        map_config |= (y & 0x03) << 2
        map_config |= x & 0x03
        self._i2c_device.write_uint8(self._address, BNO055_AXIS_MAP_CONFIG_ADDR, map_config)
        # Set the axis remap sign register value.
        sign_config = 0x00
        sign_config |= (x_sign & 0x01) << 2
        sign_config |= (y_sign & 0x01) << 1
        sign_config |= z_sign & 0x01
        self._i2c_device.write_uint8(self._address, BNO055_AXIS_MAP_SIGN_ADDR, sign_config)
        # Go back to normal operation mode.
        self._operation_mode()

    def _read_vector(self, address, count=3):
        # Read count number of 16-bit signed values starting from the provided
        return self._i2c_device.read_array(self._address, address, count, 'h')

    def read_euler(self):
        """Return the current absolute orientation as a tuple of heading, roll,
        and pitch euler angles in degrees.
        """

        heading, roll, pitch = self._read_vector(BNO055_EULER_H_LSB_ADDR)

        # Note: There appears to be an issue regarding the sign of roll.  The value being
        # returned has opposite sign of what is expected.  This was verified in the 
        # test code below where the quat_to_quler function returns the expected roll but
        # this function, read_euler, does not.  Since this is likely a error is the device
        # configuration, I am documenting and fixing it here by just changing the sign.
        roll = -roll

        return (heading/16.0, roll/16.0, pitch/16.0)

    def read_magnetometer(self):
        """Return the current magnetometer reading as a tuple of X, Y, Z values
        in micro-Teslas.
        """
        x, y, z = self._read_vector(BNO055_MAG_DATA_X_LSB_ADDR)
        return (x/16.0, y/16.0, z/16.0)

    def read_gyroscope(self):
        """Return the current gyroscope (angular velocity) reading as a tuple of
        X, Y, Z values in degrees per second.
        """
        x, y, z = self._read_vector(BNO055_GYRO_DATA_X_LSB_ADDR)
        return (x/900.0, y/900.0, z/900.0)

    def read_accelerometer(self):
        """Return the current accelerometer reading as a tuple of X, Y, Z values
        in meters/second^2.
        """
        x, y, z = self._read_vector(BNO055_ACCEL_DATA_X_LSB_ADDR)
        return (x/100.0, y/100.0, z/100.0)

    def read_linear_acceleration(self):
        """Return the current linear acceleration (acceleration from movement,
        not from gravity) reading as a tuple of X, Y, Z values in meters/second^2.
        """
        x, y, z = self._read_vector(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR)
        return (x/100.0, y/100.0, z/100.0)

    def read_gravity(self):
        """Return the current gravity acceleration reading as a tuple of X, Y, Z
        values in meters/second^2.
        """
        x, y, z = self._read_vector(BNO055_GRAVITY_DATA_X_LSB_ADDR)
        return (x/100.0, y/100.0, z/100.0)

    def read_quaternion(self):
        """Return the current orientation as a tuple of X, Y, Z, W quaternion
        values.
        """
        w, x, y, z = self._read_vector(BNO055_QUATERNION_DATA_W_LSB_ADDR, 4)
        # Scale values, see 3.6.5.5 in the datasheet.
        scale = (1.0 / (1<<14))
        return (x*scale, y*scale, z*scale, w*scale)

    def read_temp(self):
        """Return the current temperature in Celsius."""
        return self._i2c_device.read_int8(self._address, BNO055_TEMP_ADDR)


def module_test():
    from i2cdevice import I2CDevice, I2CDeviceError
    import sys
    import time
    import math

    def quat_to_euler(x, y, z, w):
        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        sinr = 2.0 * (w * x + y * z)
        cosr = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr, cosr)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny, cosy)

        return yaw, pitch, roll


    calibrate = False

    if len(sys.argv) > 1:
        if sys.argv[1] == 'cal':
            calibrate = True

    try:
        i2c = I2CDevice(I2CDevice.DEV_I2C_1)
    except I2CDeviceError as err:
        print(err)

    try:
        bno = BNO055(i2c=i2c, logger=Logger())

        bno.begin()
        
        status, self_test, error = bno.get_system_status()
        print("status: {}, self_test: {}, error: {}".format(bin(status), bin(self_test), bin(error)))

        if calibrate:
            while True:
                status = bno.get_calibration_status()
                print(status)
                if len([True for st in status if st == 3]) >= 4:
                    cal = bno.get_calibration()
                    print(cal)
                    f = open('bno055.cal', 'w')
                    f.write(str(cal))
                    f.close()
                    sys.exit()
                time.sleep(0.1)

        bno.set_calibration()
       
        now = time.time()
        next_now = now + 5
        systm, gyro, accel, mag = bno.get_calibration_status()
        while not (systm == 3 and gyro == 3 and accel == 3 and mag == 3):
            systm, gyro, accel, mag = bno.get_calibration_status()
            if time.time() > next_now:
                print('sys: {}, gyro: {}, accel: {}, mag: {}'.format(systm, gyro, accel, mag))
                next_now = time.time() + 5

        #systm, gyro, accel, mag = bno.get_calibration_status()
        print('sys: {}, gyro: {}, accel: {}, mag: {}'.format(systm, gyro, accel, mag))

        while True:
            heading, roll, pitch = bno.read_euler()
            quat = bno.read_quaternion()

            # This code was added to check the Euler angle calculations (see note in read_euler)
            # read_euler returns roll with opposite orientation, - for cw rotation, + for ccw rotation
            # quat_to_euler returns expected orientation, + for cw rotation, - for ccw rotation
            #heading1, roll1, pitch1 = quat_to_euler(*quat)

            quat_str = "x: {:6.3f}, y: {:6.3f}, z: {:6.3f}, w: {:6.3f}".format(*quat)
            euler_str = 'y: {:6.3f}, r: {:6.3f}, p: {:6.3f}\r'.format(heading, roll, pitch)
            sys.stdout.write(quat_str + ' ' + euler_str)
            sys.stdout.flush()
            time.sleep(0.1)

    except I2CDeviceError as err:
        print(err)



if __name__ == "__main__":
    module_test()

# --- EOF ---

#! /usr/bin/env python
"""
This module provides an interface for reading and writing to the i2c bus via the smbus module
It creates an abstraction that supports reading/writing numeric types and arrays of 
numeric types.
"""
from __future__ import print_function

"""
---------------------------------------------------------------------------------------------------
Imports
---------------------------------------------------------------------------------------------------
"""
# Standard Library
import sys
import struct

# Third-Party
from smbus import SMBus


"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

class I2CDeviceError(Exception):
    pass


class I2CDevice(object):

    # The device ids supported
    DEV_I2C_0 = 0
    DEV_I2C_1 = 1
    DEV_I2C_2 = 2

    SUPPORTED_DEVICES = [DEV_I2C_0, DEV_I2C_1, DEV_I2C_2]

    # The following formatting is taken from struct and maps the character designations to number of bytes in the type
    __TYPE_SIZES = {'d': 8, # double - 8 bytes
                    'f': 4, # float - 4 bytes
                    'L': 4, # uint32 - 4 bytes
                    'l': 4, # int32 - 4 bytes
                    'H': 2, # uint16 - 2 bytes
                    'h': 2, # int16 - 2 bytes
                    'B': 1, # uint8 - 1 byte
                    'b': 1  # int8 - 1 byte
                   }

    def __init__(self, device=None, bus=None, logger=None):
        """
        Initialize the I2CDevice

        Note: How a particular device id maps to an actual I2C device on the operating system is
        beyond the scope of this module.  The user will need to determine the mapping and use
        the appropriate device id.
         
        :param device: device id as seen by SMBus 
        """
        if device and device not in I2CDevice.SUPPORTED_DEVICES:
            raise I2CDeviceError("Invalid device {}, expected one of {}".format(device, I2CDevice.SUPPORTED_DEVICES))

        try:
            self._smbus = bus or SMBus(device)
        except (IOError, RuntimeError) as err:
            raise I2CDeviceError(err)

    def _read_multiple_bytes(self, address, offset, num_bytes):
        """
        Read the specified number of bytes from specified address at the specified offset.
        :param address: the specified address
        :param offset: the specified offset
        :param num_bytes: the number of bytes to read
        :return: bytes read
        """
        try:
            return self._smbus.read_i2c_block_data(address, offset, num_bytes)
        except (IOError, RuntimeError) as err:
            raise I2CDeviceError(err)

    def _write_multiple_bytes(self, address, offset, byte_values):
        """
        Write the specified bytes to specified address at the specified offset.
        :param address: the specified address
        :param offset: the specified offset
        :param byte_values: the bytes to be written
        :return: None
        """
        try:
            self._smbus.write_i2c_block_data(address, offset, list(byte_values))
        except (IOError, RuntimeError) as err:
            raise I2CDeviceError(err)

    def _create_format(self, type, endian=sys.byteorder):
        return '<'+type if endian == 'little' else '>'+type

    def read_uint8(self, address, offset):
        """
        Read uint8 value from the specified address at the specified offset.
        :param address: the specified address
        :param offset: the specified offset
        :return: the uint8 value
        """
        try:
            return self._smbus.read_byte_data(address, offset)
        except (IOError, RuntimeError) as err:
            raise I2CDeviceError(err)

    def read_int8(self, address, offset):
        """
        Read int8 value from the specified address at the specified offset.
        :param address: the specified address
        :param offset: the specified offset
        :return: the int8 value
        """

        # Note: read_byte_data returns a raw byte (unsigned), so it must
        # be converted to a signed value in order to return int8
        try:
            data = self._smbus.read_byte_data(address, offset)
        except (IOError, RuntimeError) as err:
            raise I2CDeviceError(err)
        else:
            if data > 127:
                return data - 256
            else:
                return data

    def read_uint16(self, address, offset):
        """
        Read uint16 value from the specified address at the specified offset.
        :param address: the specified address
        :param offset: the specified offset
        :return: the uint16 value
        """
        try:
            return self._smbus.read_word_data(address, offset)
        except (IOError, RuntimeError) as err:
            raise I2CDeviceError(err)

    def read_int16(self, address, offset):
        """
        Read int16 value from the specified address at the specified offset.
        :param address: the specified address
        :param offset: the specified offset
        :return: the int16 value
        """
        try:
            return self._smbus.read_word_data(address, offset)
        except (IOError, RuntimeError) as err:
            raise I2CDeviceError(err)

    def read_uint32(self, address, offset):
        """
        Read uint32 value from the specified address at the specified offset.
        :param address: the specified address
        :param offset: the specified offset
        :return: the uint32 value
        """
        fmt = self._create_format('L')
        bytes = self._read_multiple_bytes(address, offset, I2CDevice.__TYPE_SIZES['L'])
        return struct.unpack(fmt, str(bytearray(bytes)))[0]

    def read_int32(self, address, offset):
        """
        Read int32 value from the specified address at the specified offset.
        :param address: the specified address
        :param offset: the specified offset
        :return: the int32 value
        """
        fmt = self._create_format('l')
        bytes = self._read_multiple_bytes(address, offset, I2CDevice.__TYPE_SIZES['l'])
        return struct.unpack(fmt, str(bytearray(bytes)))[0]

    def read_float(self, address, offset):
        """
        Read float value from the specified address at the specified offset.
        :param address: the specified address
        :param offset: the specified offset
        :return: the float value
        """
        fmt = self._create_format('f')
        values = self._read_multiple_bytes(address, offset, I2CDevice.__TYPE_SIZES['f'])
        return struct.unpack(fmt, str(bytearray(values)))[0]

    def read_array(self, address, offset, num_values, type, endian=sys.byteorder):
        """
        Read array of specified value and type from the specified address from the specified offset. 
        :param address: the specified address
        :param offset: the specified offset
        :param num_values: the number of values to read
        :param type: the specified type of the value
        :param endian: the endianess value
        :return: the value array
        """
        # Calculate number of bytes to read
        #   - num_values is the number of values to read
        #   - num_bytes is num_values * size of each value
        num_bytes = num_values * I2CDevice.__TYPE_SIZES[type]

        # It turns out that reading i2c block data is not supported on all Raspberry Pi's (probably a OS/driver difference)
        # The Pi 2 running Jessie doesn't support i2c (i2cget with no arguments shows no 'i' option)
        # The Pi 3 running Jessie does support i2c (i2cget with no argument shows 'i' option)
        # So, we need to support both options
        values = self._read_multiple_bytes(address, offset, num_bytes)

        # Create a format specifier based on the number of values requested.
        # All of the values will be read as the same type, e.g., all floats, all long, etc
        # The format specifies the number of float values to convert
        format = '%s%s' % (num_values, type)

        # Match the endianess of the request.  Default is platform endianess
        # struct provides a format specifier for endianess
        fmt = self._create_format(format, endian)
        return list(struct.unpack(fmt, str(bytearray(values))))

    def write_uint8(self, address, offset, value):
        """
        Write uint8 value to the specified address at the specified offset. 
        :param address: the specified address
        :param offset: the specified offset
        :param value: the value to be written
        :return: None
        """
        try:
            self._smbus.write_byte_data(address, offset, value)
        except (IOError, RuntimeError) as err:
            raise I2CDeviceError(err)

    def write_uint16(self, address, offset, value):
        """
        Write uint16 value to the specified address at the specified offset.
        :param address: the specified address
        :param offset: the specified offset
        :param value: the value to be written
        :return: None
        """
        try:
            self._smbus.write_word_data(address, offset, value)
        except (IOError, RuntimeError) as err:
            raise I2CDeviceError(err)

    def write_int16(self, address, offset, value):
        """
        Write int16 value to the specified address at the specified offset.
        :param address: the specified address
        :param offset: the specified offset
        :param value: the value to be written
        :return: None
        """
        try:
            self._smbus.write_word_data(address, offset, value)
        except (IOError, RuntimeError) as err:
            raise I2CDeviceError(err)

    def write_uint32(self, address, offset, value):
        """
        Write uint32 value to the specified address at the specified offset.
        :param address: the specified address
        :param offset: the specified offset
        :param value: the value to be written
        :return: None
        """
        fmt = self._create_format('L')
        bytes = bytearray(struct.pack(fmt, value))
        self._write_multiple_bytes(address, offset, bytes)

    def write_float(self, address, offset, value):
        """
        Write float value to the specified address at the specified offset.        
        :param address: the specified address
        :param offset: the specified offset
        :param value: the value to be written
        :return: None
        """
        fmt = self._create_format('f')
        bytes = bytearray(struct.pack(fmt, value))
        self._write_multiple_bytes(address, offset, bytes)

    def write_array(self, address, offset, values, type, endian=sys.byteorder):
        """
        Write the array of the specified values to the specified address at the specified offset.
        :param address: the specified address
        :param offset: the specified offset
        :param values: the specified values
        :param type: the specified value type
        :param endian: the specified endianess of the value
        :return: None
        """
        # Convert each value to its byte representation and place into a bytearray before writing to the bus
        # Note: struct.pack returns a string representation of the value.  For a 1-byte value, it is a
        # string representation of 1 byte, for a 2 or 4 byte value, it is a 2-byte of 4-byte representation
        # Therefore, it is necessary to concatentate the strings before converting to a bytearray

        fmt = self._create_format(type, endian)
        byte_values = ''.join([struct.pack(fmt, value) for value in values])
        self._write_multiple_bytes(address, offset, bytearray(byte_values))


def module_test():
    print(__name__, " Module Test running ...")
    i2c = I2CDevice(I2CDevice.DEV_I2C_1)
    for i in range(16):
        print("{:02x} - {:02x}".format(i, i2c.read_uint8(0x08, i)))
    data = i2c.read_array(0x08, 0, 16, 'B')
    print(data)

    print(__name__, " Module Test complete.")


if __name__ == "__main__":
    module_test()


# --- EOF ---

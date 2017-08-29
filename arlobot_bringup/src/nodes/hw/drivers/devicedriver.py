#!/usr/bin/env python
"""
---------------------------------------------------------------------------------------------------
File: devicedriver.py

Description: This module contains the class definition for the DeviceDriver base class.  The 
DeviceDriver class inherits from StoppableThread and contains a default generator implementation
to retrieve data from a device and pass it to a provided callback.

Author: Tim Slator
Date: 11APR17
License: MIT
---------------------------------------------------------------------------------------------------
"""
from __future__ import print_function

"""
---------------------------------------------------------------------------------------------------
Imports
---------------------------------------------------------------------------------------------------
"""

# Standard
import time

# Third-Party
# None

# Project
from stoppablethread import StoppableThread
from utils.logger import Logger


"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

class DeviceDriver(StoppableThread):
    """
    Base class for device drivers inheriting from StoppableThread
        - Instantiates the device
        - Create a generator object to retreive data/messages from devices
        - Defines default thread function, work, to pull messages from the generator
    """
    def __init__(self, name, device, callback, logger=None):
        super(DeviceDriver, self).__init__(name, logger=logger)

        self._name = name
        self._msgs = self._gen_msg()
        self._callback = callback
        self._device = device
        self._logger = logger or Logger()

    def _gen_msg(self):
        while True:
            message = self._device.recv()
            self._logger.debug("{} received msg: ".format(self._name, message))
            if message:
                yield message

            if not self._running.is_set():
                self._logger.debug("{} generator stopped".format(self._name))
                raise StopIteration
        
    def work(self):
        for msg in self._msgs:
            if self._callback:
                self._callback(msg)

        self._logger.debug("{} stop iteration raised".format(self._name))

    def send(self, data):
        self._logger.debug("{} sending data".format(self._name, data))
        self._device.send(data)

    def stop(self):
        self._logger.debug("{} DeviceDriver stop".format(self._name))
        super(DeviceDriver, self).stop()


def module_test():
    import time
    class Device(object):
        counter = 0
        def send(self, msg):
            print(msg)

        def recv(self):
            time.sleep(0.1)
            count = Device.counter
            Device.counter += 1
            return count

    def callback(msg):
        print(msg)


    print("DeviceDriver Module Test")

    dd = DeviceDriver("test", Device(), callback)
    print("Started ...")
    dd.start()
    time.sleep(1)
    dd.stop()
    print("Stopped!")


if __name__ == "__main__":
    module_test()


# --- EOF ---

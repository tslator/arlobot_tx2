#!/usr/bin/env python
"""
---------------------------------------------------------------------------------------------------
File: stoppablethread.py
Description: 
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
from threading import Thread, Event
import time

# Third-Party
# None

# Project
from common import Logger


"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""


class StoppableThreadError(Exception):
    pass


class StoppableThread(Thread):
    def __init__(self, name, event=None, logger=None):
        super(StoppableThread, self).__init__(name=name)
        self._running = event or Event()
        self._running.clear()
        self._name = self.getName()
        self._logger = logger or Logger()

    def run(self):
        self._logger.debug("{} starting run".format(self._name))
        while self._running.is_set():
            self.work()

        self._logger.debug("{} exiting run".format(self._name))
        
    def work(self):
        self._logger.debug("{} empty work".format(self._name))

    def start(self):
        self._logger.debug("{} starting".format(self._name))
        self._running.set()
        self.setDaemon(True)
        super(StoppableThread, self).start()

    def stop(self):
        start = time.time()
        self._logger.debug("{} stopping ...".format(self._name))

        self._running.clear()

        #TODO: There are two issues that need to be resolved:
        # 1. When shutting down the sensor node, the sensor driver doesn't quit which causes join to hang.  That's seems
        #    to be due to the blocking recv call.  A call to shutdown was added but that doesn't appear to have resolved
        #    the issue.  Right now, CAN is not being used and it is possible that when CAN messages are received, the
        #    behavior may change.  A short term fix is to add a timeout to the join call.  Not ideal but since CAN isn't
        # 2. Sometimes an object fails (any exception) before its driver can be started which results in shutdown
        #    processing and a call to stop.  If the driver thread wasn't started then join throws an exception.  While
        #    not catastrophic, it would be nice to have better and understanding and control for this case.
        try:
            self.join(1.0)
        except RuntimeError as err:
            self._logger.warn(err)

        self._logger.debug("{} stopped (time: {:01.3})".format(self._name, time.time() - start))
        
        
def module_test():
    import time

    st = StoppableThread('test thread')
    st.start()    
    time.sleep(5)
    st.stop()


if __name__ == "__main__":
    module_test()

# --- EOF ---

#!/usr/bin/env python
"""
---------------------------------------------------------------------------------------------------
File: pid.py

Description: Provides standard PID (proportional, integral, derivative) controller

Author: Tim Slator
Date: 11APR17
License: MIT
--------------------------------------------------------------------------------------------------
"""
from __future__ import print_function

# Standard Library
import sys
import math

from .transforms import constrain

"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

class PID:
    """
    Implements basic PID controller
    """
    def __init__(self, name, kp=1, ki=0, kd=0, min=0.0, max=1.0, sample_time=1.0):
        self._name = name
        self._gains = kp, ki, kd
        self._min = min
        self._max = max

        self._kp = kp * sample_time
        self._ki = ki
        self._kd = kd / sample_time

        self._last_error = 0.0
        self._error_sum = 0.0
        self._target = 0.0
        self._target_sign = 0.0

        self._state = tuple()

    def _preprocess(self, value):
        sign = math.copysign(1.0, value)
        constrained = constrain(abs(value), self._min, self._max)
        normal = constrained / self._max

	return sign, normal

    def set_target(self, target):
        self._target_sign, self._target = self._preprocess(target)

    def update(self, measured):
        measured_sign, measured_normal = self._preprocess(measured)

        error = self._target - measured_normal

        e_dot = error - self._last_error
        self._error_sum += error

        output = self._kp * error + self._kd * e_dot + self._ki * self._error_sum

        # Compensate for sign crossings
        if measured_sign < 0.0:
            output *= measured_sign
        elif self._target_sign < 0.0:
            output *= self._target_sign

        # Denormalize
        output *= self._max

        self._state = error, self._last_error, e_dot, self._error_sum, output

        self._last_error = error

        return output

    def print(self, out=sys.stdout.write):
        state = ', '.join(['{}:{:02.3f}'.format(*s) for s in zip(('e', 'l', 'd', 's', 'o'), self._state)])
        gains = ', '.join(['{}:{:02.3f}'.format(*g) for g in zip(('kp', 'ki', 'kd'), self._gains)])
        out("{} - state: {} <-> gains: {}\n".format(self._name, state, gains))


def module_test():
    print(__name__, "Module Test")

    import time

    p = PID('pid', 1.0, 0.0, 0.0, 0.0, 1.3, 0.1)


    for i in range(100):
        time.sleep(0.1)
        if i < 25:
            p.set_target(1.0)
            p.update(i*0.1)
            print("Target: 1.0, Input: {}".format(i*0.1))
        elif 25 <= i < 50:
            p.set_target(0.0)
            p.update(-(50 - i)*0.1)
            print("Target: 0.0, Input: {}".format(-(50-i)*0.1))
        elif 50 <= i < 75:
            p.set_target(-1.0)
            p.update(-(75 -i)*0.1)
            print("Target: -1.0, Input: {}".format(-(75-i)*0.1))
        else: 
            p.set_target(0.0)
            p.update((100 - i)*0.1)
            print("Target: 0.0, Input: {}".format((100-i)*0.1))
                    
        p.print()


if __name__ == "__main__":
    module_test()

# --- EOF ---

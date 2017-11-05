#!/usr/bin/env python
from __future__ import print_function
"""
---------------------------------------------------------------------------------------------------
File: teleopnode.py

Description: Provides mapping between joy_node and xbox key presses.

Author: Tim Slator
Date: 23APR17
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

# Third-Party
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# Project
import basenode


'''
Details of the buttons and axis fields of the Joy message

Table of index number of /joy.buttons:

Index - Button name on the actual controller
0 - A
1 - B
2 - X
3 - Y
4 - LB
5 - RB
6 - back
7 - start
8 - power
9 - Button stick left
10 - Button stick right

Table of index number of /joy.axis:

Index - Axis name on the actual controller
0 - Left/Right Axis stick left
1 - Up/Down Axis stick left
2 - Left/Right Axis stick right
3 - Up/Down Axis stick right
4 - RT
4 - LT
6 - cross key left/right
7 - cross key up/down

http://wiki.ros.org/joy#joy_node.py
'''


Buttons = namedtuple('Buttons', 'A B X Y LB RB LT RT back start power button_stick_left button_stick_right leftpress_left leftpress_right leftpress_up leftpress_down ')
Axes = namedtuple('Axes', 'leftright_stick_left updown_stick_left leftright_stick_right updown_stick_right')
Velocity = namedtuple('Velocity', 'angular linear')


class TeleopNodeError(Exception):
    pass


class TeleopNode(basenode.BaseNode):
    """
    Teleoperation node.  Converts xbox key commands
    """
    def __init__(self, debug=False):
        super(TeleopNode, self).__init__(name='TeleopNode', debug=debug)

        rospy.loginfo("TeleopNode init")

        self._linear_scale = rospy.get_param("Linear Scale", 1.0)
        self._angular_scale = rospy.get_param("Angular Scale", 1.0)

        self._pub = rospy.Publisher('cmd_vel', Twist, queue_size=2)
        self._sub = rospy.Subscriber('joy', Joy, self._joy_callback)

        rospy.Timer(rospy.Duration(0.1), self._twist_pub_callback)

        self._deadman_pressed = True
        self._zero_twist_published = False
        self._last_published = Twist()

    def _joy_callback(self, joy):
        buttons = Buttons(*joy.buttons)
        axes = Axes(*joy.axes)
        velocity = Velocity(*joy.axes[:2])

        twist = Twist()
        twist.angular.z = velocity.angular #self._angular_scale * velocity.angular
        twist.linear.x = velocity.linear #self._linear_scale * velocity.linear
        log_format = 'Deadman: {}, Linear: {:.3f}, Angular: {:.3f}'.format(buttons.LB,  twist.linear.x, twist.angular.z)
        rospy.logdebug(log_format)
        self._last_published = twist
        self._deadman_pressed = bool(buttons.LB)

    def _twist_pub_callback(self, event):
        if self._deadman_pressed:
            self._pub.publish(self._last_published)
            self._zero_twist_published = False
        elif not self._deadman_pressed and not self._zero_twist_published:
            self._last_published = Twist()
            self._pub.publish(self._last_published)
            self._zero_twist_published = True

    def start(self):
        pass

    def loop(self):
        rospy.spin()

    def shutdown(self):
        pass

if __name__ == "__main__":
    try:
        telenode = TeleopNode(debug=False)
    except TeleopNodeError as err:
        rospy.fatal("Unable to instantiate TeleopNode - {}".format(err))
    else:
        try:
            telenode.start()
            telenode.loop()
        except rospy.ROSInterruptException as err:
            rospy.fatal("Interrupt Exception raised in TeleopNode - {}".format(err))
            telenode.shutdown()

# --- EOF ---

#!/usr/bin/env python
from __future__ import print_function


# Standard Library
import itertools
from collections import namedtuple

# Third Party
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry

# Project
from arlobot_ws.src.common.basenode import BaseNode


Buttons = namedtuple('Buttons', 'A B X Y LB RB back start power button_stick_left button_stick_right')
Axes = namedtuple('Axes', 'leftright_stick_left updown_stick_left leftright_stick_right updown_stick_right RT LT leftright_cross_key updown_crosskey')


# Create a test node that uses the XBOX 360 controller to signal different types of tests/calibrations
#  - Move forward 1 meter
#  - Move backward 1 meter
#  - Left rotate 360 degrees
#  - Right rotate 360 degrees
#  - Move in 1 meter box (left turn)
#  - Move in 1 meter box (right turn)
#  - Move in circle (1 meter radius) ccw
#  - Move in circle (1 meter radius) cw


class TestNodeError(Exception):
    pass


class TestNode(BaseNode):
    def __init__(self):
        super(TestNode, self).__init__(name='TestNode', debug=True)

        self._joy_sub = rospy.Subscriber('joy', Joy, self._joy_callback, queue_size=1)
        self._odom_sub = rospy.Subscriber('Odometry', Odometry, self._odom_callback, queue_size=1)
        self._pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self._test_vector = {
            'fblr':
            [
            # Forward, Backward, Left Turn, Right Turn

            Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=0.064, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
            Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=-0.064, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
            Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.323)),
            Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-0.323)),

            ],

            'fb':
            [
            # Forward, Backward

            Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=0.064, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
            Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=-0.064, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            ],

            'fbnt':
            [
                # Forward, Backward (no transition)

                Twist(linear=Vector3(x=0.064, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
                Twist(linear=Vector3(x=-0.064, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            ],

            'lrnt':
            [
                # Left Turn, Right Turn (no transition)
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.323)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-0.323)),
            ],

            'fblrnt':
            [
                Twist(linear=Vector3(x=0.064, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
                Twist(linear=Vector3(x=-0.064, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.323)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-0.323)),
            ],

            'tf':
            [
            # Tracking Forward
            #   0.064 / 5 = 0.0128
            Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=0.0128, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=2*0.0128, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=3*0.0128, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=4*0.0128, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=5*0.0128, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=4 * 0.0128, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=3 * 0.0128, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=2 * 0.0128, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=0.0128, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
            ],

            'tb':
            [
            # Tracking Backward
            #   0.064 / 5 = 0.0128
            Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=-0.0128, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=-2 * 0.0128, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=-3 * 0.0128, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=-4 * 0.0128, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=-5 * 0.0128, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=-4 * 0.0128, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=-3 * 0.0128, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=-2 * 0.0128, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            Twist(linear=Vector3(x=-0.0128, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
            ],

            'la':
            [
                Twist(linear=Vector3(x=0.064, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.323)),
                Twist(linear=Vector3(x=0.064, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-0.323)),
                Twist(linear=Vector3(x=0.064, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

                Twist(linear=Vector3(x=-0.064, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.323)),
                Twist(linear=Vector3(x=-0.064, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-0.323)),
                Twist(linear=Vector3(x=-0.064, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

            ],

            'lhb':
            [
                Twist(linear=Vector3(x=0.02, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-0.0618)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
                Twist(linear=Vector3(x=0.02, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-0.0618)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),

                Twist(linear=Vector3(x=0.02, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-0.0618)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
                Twist(linear=Vector3(x=0.02, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-0.0618)),
                Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
            ]

        }

        self._pub_msg = Twist()
        self._test_vector_index = 0
        self._vel_cycle = None
        self._rate = rospy.Rate(10)

        self._vel_cycle = itertools.cycle(self._test_vector['lhb'])

    def _joy_callback(self, msg):
        key = 'fblr'
        #key = 'tf'
        #key = 'tb'

        buttons = Buttons(*msg.buttons)
        axes = Axes(*msg.axes)

        #rospy.loginfo(str(buttons))
        #rospy.loginfo(str(axes))

        if buttons.A:
            key = 'fblr'
            rospy.loginfo("Cycling fblr")
            self._pub_msg = Twist()
            rospy.loginfo("Creating new cycle")
            del self._vel_cycle
            self._vel_cycle = itertools.cycle(self._test_vector[key])
        elif buttons.B:
            key = 'tf'
            rospy.loginfo("Cycling tf")
            self._pub_msg = Twist()
            rospy.loginfo("Creating new cycle")
            del self._vel_cycle
            self._vel_cycle = itertools.cycle(self._test_vector[key])
        elif buttons.X:
            key = 'tb'
            rospy.loginfo("Cycling tb")
            self._pub_msg = Twist()
            rospy.loginfo("Creating new cycle")
            del self._vel_cycle
            self._vel_cycle = itertools.cycle(self._test_vector[key])

        elif axes.leftright_cross_key == 1.0:
            rospy.loginfo("Move forward 1 meter")
        elif axes.leftright_cross_key == -1.0:
            rospy.loginfo("Move backward 1 meter")
        elif axes.updown_crosskey == 1.0:
            rospy.loginfo("Turn CCW 360 degrees")
        elif axes.updown_crosskey == -1.0:
            rospy.loginfo("Turn CW 360 degrees")

    def _odom_callback(self, msg):
        rospy.loginfo("TestNode: " + msg)

    def _next_twist_msg(self, event):
        if self._vel_cycle:
            self._pub_msg = next(self._vel_cycle)

    def loop(self):
        while not rospy.is_shutdown():

            #rospy.loginfo("Publishing: {}".format(self._pub_msg))
            self._pub.publish(self._pub_msg)
            self._rate.sleep()

        self.shutdown()

    def shutdown(self):
        pass

    def start(self):
        rospy.Timer(rospy.Duration(5), self._next_twist_msg)


if __name__ == "__main__":
    try:
        test = TestNode()
    except TestNodeError as err:
        rospy.fatal("Unable to instantiate TestNode - {}".format(err))
    else:
        try:
            test.start()
            test.loop()
        except rospy.ROSInterruptException as err:
            rospy.fatal("Interrupt Exception raised in HALNode - {}".format(err))
            test.shutdown()

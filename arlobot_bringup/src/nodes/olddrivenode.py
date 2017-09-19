#!/usr/bin/env python
"""
----------------------------------------------------------------------------------------------------
File: olddrivenode.py

Description: Provides implementation of the DriveNode responsible for:
                  * driving the motors by converting Twist messages into HALSpeedOut messages
                  * publishing odometry by converting HAL messages from PsocHw and ImuHw
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
import math
from collections import namedtuple, deque
from statistics import mean

# Third-Party
import rospy
from geometry_msgs.msg import Twist

# Project
from basenode import BaseNode
from arlobot_ws.src.arlobot_tx2.common.motion import ensure_w, uni_max
from arlobot_bringup.msg import HALSpeedIn, HALEulerIn, HALPositionIn, HALHeadingIn
from pubs.odompub import OdometryPublisher
from pubs.halpub import HALSpeedOutPublisher
from utils.pid import PID
from arlobot_ws.src.arlobot_tx2.common.transforms import isclose, rpm_to_rps, calc_vel_inc, constrain, normalize_vector
from hw.messages import SpeedData

STATUS_FMT = '''
Track Width: {:6.3f}
Radius: {:6.3f}
Diameter: {:6.3f}
Circumference: {:6.3f}
Max Wheel Angular Velocity: {:6.3f}
Max Angular Velocity: {:6.3f}
Max Linear Velocity: {:6.3f}
Max Angular Accel: {:6.3f}
Max Linear Accel: {:6.3f}
'''

# Named tuples to hold maximum values for linear and angular motion.
LinearMax = namedtuple('LinearMax', 'forward backward accel decel')
AngularMax = namedtuple('AngularMax', 'ccw cw accel decel wheel')


"""
---------------------------------------------------------------------------------------------------
Classes
---------------------------------------------------------------------------------------------------
"""

#---------------------------------------------------------------------------------------------------


class Velocity(object):
    def __repr__(self):
        return "Velocity(linear={:02.3f}, angular={:02.3f})".format(self.linear, self.angular)

    def __init__(self, v=0.0, w=0.0):
        self.linear = v
        self.angular = w

    def set(self, v, w):
        self.linear = v
        self.angular = w

    def zero(self):
        self.linear = 0.0
        self.angular = 0.0

#---------------------------------------------------------------------------------------------------

class OdometryState(object):
    """
    Helper class to hold odometry-related values
    """
    def __repr__(self):
        return "OdometryState(velocity={:02.3f}, cdist={:02.3f}, heading={:02.3f}".format(
                    self.velocity,
                    self.cdist,
                    self.heading)

    def __init__(self):
        self._speed_ready = False
        self._dist_ready = False
        self._euler_ready = False
        self.velocity = Velocity()
        self.x = 0.0
        self.y = 0.0
        self.cdist = 0.0
        self.heading = 0.0
        self.quat = None

    def is_ready(self):
        return self._speed_ready and self._dist_ready and self._euler_ready

    def set_velocity(self, v, w):
        self.velocity.set(v, w)
        self._speed_ready = True

    def set_position(self, x, y):
        self.x = x
        self.y = y
        self.cdist = math.sqrt(x**2 + y**2)
        self._dist_ready = True

    def set_orientation(self, yaw=0.0, pitch=0.0, roll=0.0):
        # Limit yaw to -pi/2 to pi/2
        self.heading = yaw if yaw <= math.pi else yaw - (2 * math.pi)
        self._euler_ready = True

#---------------------------------------------------------------------------------------------------

class VelocityState(object):
    """
    Helper class to hold state variables
    """
    def __repr__(self):
        return "VelocityState(target_velocity={}, last_velocity={}".format(self.target_velocity, self.last_velocity)

    def __init__(self, target=None, last=None):
        self.target_velocity = target or Velocity()
        self.last_velocity = last or Velocity()

    def zero_target(self):
        self.target_velocity.zero()

    def zero_last(self):
        self.last_velocity.zero()

    def zero_all(self):
        self.zero_target()
        self.zero_last()

#---------------------------------------------------------------------------------------------------

class CommandVelocityState(object):
    """
    Helper class to track command velocity
    """
    def __repr__(self):
        return "CommandVelocityState(velocity={:02:3f}, received={}, period={:02:3f}, last_time={:02:3f},\nhistory={})".format(
            self.velocity,
            self._received,
            self.period,
            self.last_time,
            self._history
        )

    def __init__(self, depth=5):
        self._history = deque([0.1]*depth, depth)
        self._received = False
        self.period = 0.0
        self.last_time = rospy.Time.now()
        self.velocity = Velocity()

    def add_delta(self, now):
        self._received = True

        delta = (now - self.last_time).to_sec()
        self._history.append(delta)
        self.period = mean(self._history)

        self.last_time = rospy.Time.now()

    def is_recent(self, num_msgs, max_time=0.5):
        allowed_num_msgs = min(num_msgs, len(self._history))

        delta = (rospy.Time.now() - self.last_time).to_sec()
        self._received = delta <= min(allowed_num_msgs * self.period, max_time)

        return self._received

    def is_active(self):
        result = self._received and self.period > 0.0
        if not result:
            self.velocity.zero()

        return result

#---------------------------------------------------------------------------------------------------


class DriveNodeError(Exception):
    pass


class DriveNode(BaseNode):
    """
    The DriveNode is responsible for:
        * accepting Twist messages to be sent to the motor driver.
        * ensuring max linear/angular velocity and applying acceleration limits.
        * reading speed, distance and heading data from the hardware drivers, calculating and publishing
          odometry.
    """
    def __init__(self, debug=False):
        super(DriveNode, self).__init__(name='DriveNode', debug=debug)


        """
        ------------------------------------------------------------------------------------------
        Publishers
        ------------------------------------------------------------------------------------------
        """

        # Publisher of speed to motor driver and odometry to other nodes
        self._speedout_pub = HALSpeedOutPublisher()
        self._odom_pub = OdometryPublisher()

        """
        ------------------------------------------------------------------------------------------
        Parameters
        ------------------------------------------------------------------------------------------
        """

        # Robot physical dimensions
        self._track_width = rospy.get_param('Track Width', 0.3937)
        self._wheel_radius = rospy.get_param('Wheel Radius', 0.0785)

        # Main loop rate in hz
        drive_node_rate = rospy.get_param('Drive Node Rate', 10)
        self._loop_rate = rospy.Rate(drive_node_rate)
        self._loop_period = 1.0/float(drive_node_rate)

        max_wheel_rpm = rospy.get_param('Max Wheel RPM', 95.0)
        max_wheel_ang_vel = rpm_to_rps(max_wheel_rpm, self._wheel_radius)

        # Calculate max unicycle velocities based on wheel maximum, track width and wheel radius
        max_lin_vel, max_ang_vel = uni_max(max_wheel_ang_vel, self._track_width, self._wheel_radius)

        max_angular_accel = rospy.get_param('Max Angular Accel', 0.15)
        max_angular_decel = rospy.get_param('Max Angular Decel', 0.15)

        self._angular_max = AngularMax(ccw=max_ang_vel,
                                       cw=-max_ang_vel,
                                       accel=max_angular_accel,
                                       decel=-max_angular_decel,
                                       wheel=max_ang_vel)

        max_linear_accel = rospy.get_param('Max Linear Accel', 0.035)
        max_linear_decel = rospy.get_param('Max Linear Decel', 0.035)

        self._linear_max = LinearMax(forward=max_lin_vel,
                                     backward=-max_lin_vel,
                                     accel=max_linear_accel,
                                     decel=-max_linear_decel)

        # PID controllers
        linear_kp = rospy.get_param('Linear Gain Kp', 0.8)
        linear_ki = rospy.get_param('Linear Gain Ki', 0.0)
        linear_kd = rospy.get_param('Linear Gain Kd', 0.0)
        self._lin_pid = PID('linear', linear_kp, linear_ki, linear_kd, 0.0, max_lin_vel, self._loop_period)

        angular_kp = rospy.get_param('Angular Gain Kp', 0.8)
        angular_ki = rospy.get_param('Angular Gain Ki', 0.0)
        angular_kd = rospy.get_param('Angular Gain Kd', 0.0)
        self._ang_pid = PID('angular', angular_kp, angular_ki, angular_kd, 0.0, max_ang_vel, self._loop_period)

        # State variable storage
        self._vel_state = VelocityState()
        self._odometry = OdometryState()
        self._cmd_vel_state = CommandVelocityState()

        rospy.loginfo(STATUS_FMT.format(
            self._track_width,
            self._wheel_radius,
            self._wheel_radius * 2,
            self._wheel_radius * 2 * math.pi,
            max_wheel_ang_vel,
            max_ang_vel,
            max_lin_vel,
            max_angular_accel,
            max_linear_accel
        ))

        """
        -------------------------------------------------------------------------------------------
        Subscribers
        -------------------------------------------------------------------------------------------
        """

        # Note: Declare subscribers at the end to prevent premature callbacks

        # Subscriber for Twist messages
        self._twist_sub = rospy.Subscriber("cmd_vel", Twist, self._twist_cmd_cb, queue_size=1)
        # Subscriber for motor driver speed, distance, and orientation messages
        self._speedin_sub = rospy.Subscriber("HALSpeedIn", HALSpeedIn, self._speedin_cb, queue_size=1)
        self._positionin_sub = rospy.Subscriber("HALPositionIn", HALPositionIn, self._positionin_cb, queue_size=1)
        self._headingin_sub = rospy.Subscriber("HALHeadingIn", HALHeadingIn, self._headingin_cb, queue_size=1)
        self._eulerin_sub = rospy.Subscriber("HALEulerIn", HALEulerIn, self._eulerin_cb, queue_size=1)


    def _twist_cmd_cb(self, msg):
        """
        Callback registered to receive cmd_vel messages

        Averages the receipt times of cmd_vel messages for use in velocity processing
        Stores velocity for later processing

        :param msg: the cmd_vel message
        :return: None
        """
        self._cmd_vel_state.add_delta(rospy.Time.now())
        self._cmd_vel_state.velocity.set(v=msg.linear.x, w=msg.angular.z)

    def _speedin_cb(self, msg):
        """
        Callback registered to receive HALSpeedIn messages

        Receives and stores unicycle speed

        :param msg: the HALSpeedIn message (see HALSpeedIn.msg)
        :return: None
        """
        self._odometry.set_velocity(msg.linear, msg.angular)

    def _positionin_cb(self, msg):
        """
        Callback registered to receive HALPositionIn messages

        Receives and stores x/y position

        :param msg: the HALPositionIn message (see HALPositionIn.msg)
        :return: None
        """
        self._odometry.set_position(msg.x, msg.y)

    def _headingin_cb(self, msg):
        """

        :param msg:
        :return:
        """
        self._odometry.set_orientation(yaw=msg.heading)

    def _eulerin_cb(self, msg):
        """
        Callback registered to receive HALEulerIn messages

        Receives and stores roll, pitch and yaw

        :param msg: the HALEulerIn message
        :return: None
        """
        #self._odometry.set_orientation(msg.roll, msg.pitch, msg.yaw)

    def _publish_odom(self):
        """
        Publishes odometry on the odom topic

        :return: None
        """
        if self._odometry.is_ready():
            self._odom_pub.publish(self._odometry.x,
                                   self._odometry.y,
                                   self._odometry.velocity.linear,
                                   self._odometry.velocity.angular,
                                   self._odometry.heading)

    def _velocity_has_changed(self):
        return (not isclose(self._vel_state.target_velocity.linear, self._vel_state.last_velocity.linear) or
                not isclose(self._vel_state.target_velocity.angular, self._vel_state.last_velocity.angular))

    def _safety_check(self):
        """
        Perform safety/sanity checking on the command velocity before passing through the velocity pipeline

        if the command velocity is active (we're getting velocity commands) and command velocity is recent
            - constrain the commanded velocity to mechanical and user-defined limits
            - ensure requested angular velocity can be achieved
            - store in target velocity
        if the command velocity is stale or not being received at all
            - zero out target velocity

        :return: None
        """

        # TODO-Add a velocity check for 'too' large velocity change with resulting adjustment that is proportional

        if self._cmd_vel_state.is_active():

            if self._cmd_vel_state.is_recent(num_msgs=5, max_time=2.0):

                v_initial = self._cmd_vel_state.velocity.linear
                w_initial = self._cmd_vel_state.velocity.angular

                # Ensure linear and angular velocity are within mecahnical and user-defined ranges
                v = constrain(v_initial, self._linear_max.backward, self._linear_max.forward)
                w = constrain(w_initial, self._angular_max.cw, self._angular_max.ccw)

                # Ensure the specified angular velocity can be achieved.
                #
                # Even though velocity is constrained within defined limits, it is necessary to check that the resulting
                # velocities are achievable.  For example if both wheels are moving at the maximum linear velocity and
                # an angular velocity is specified, it is necessary to reduce the linear velocity in order to achieve
                # the angular velocity.

                # Note: Will adjust linear velocity if necessary
                v, w = ensure_w(v, w, self._track_width, self._wheel_radius, self._angular_max.wheel)

                rospy.loginfo("SafetyCheck: initial {:.3f} {:.3f}, constrained {:.3f} {:.3f}".format(v_initial, w_initial, v, w))

                self._vel_state.target_velocity.set(v, w)

            else:
                rospy.logwarn("No message received for {} msgs or {} secs".format(5, 1.0))
                self._vel_state.zero_target()

        else:
            rospy.logwarn("No command velocity active")
            self._vel_state.zero_target()

    def _limit_accel(self):
        lv_initial = self._vel_state.last_velocity
        tv_initial = self._vel_state.target_velocity

        # Calculate the desired and max velocity increments for linear and angular velocities
        v_inc, max_v_inc = calc_vel_inc(self._vel_state.target_velocity.linear,
                                        self._vel_state.last_velocity.linear,
                                        self._linear_max.accel,
                                        self._linear_max.decel,
                                        self._loop_period)

        w_inc, max_w_inc = calc_vel_inc(self._vel_state.target_velocity.angular,
                                        self._vel_state.last_velocity.angular,
                                        self._angular_max.accel,
                                        self._angular_max.decel,
                                        self._loop_period)

        # Create normalized vectors for desired (v_inv/w_inc) and maximum (max_v_inc/max_w_inc) velocity increments
        Av, Aw = normalize_vector(v_inc, w_inc)
        Bv, Bw = normalize_vector(max_v_inc, max_w_inc)

        # Use the angle between the vectors to determine which increment will dominate
        theta = math.atan2(Bw, Bv) - math.atan2(Aw, Av)

        if theta < 0.0:
            try:
                max_v_inc = (max_w_inc * math.fabs(v_inc))/math.fabs(w_inc)
            except ZeroDivisionError:
                pass
        else:
            try:
                max_w_inc = (max_v_inc * math.fabs(w_inc))/math.fabs(v_inc)
            except ZeroDivisionError:
                pass

        # Calculate the velocity delta to be applied
        v_delta = math.copysign(1.0, v_inc) * min(math.fabs(v_inc), math.fabs(max_v_inc))
        w_delta = math.copysign(1.0, w_inc) * min(math.fabs(w_inc), math.fabs(max_w_inc))

        # Apply the velocity delta to the last velocity
        self._vel_state.target_velocity.linear = self._vel_state.last_velocity.linear + v_delta
        self._vel_state.target_velocity.angular = self._vel_state.last_velocity.angular + w_delta

        rospy.logdebug("LimitAccel - initial {} {}, resulting {} {}".format(
            lv_initial,
            tv_initial,
            self._vel_state.last_velocity,
            self._vel_state.target_velocity))

    def _track_velocity(self):
        """
        Track linear and angular velocity using PID controller
        Note: Only track when command velocity is active; otherwise, zero out published velocity

        :return:
        """
        rospy.logdebug("TV Entry - O: {}, T: {}, L: {}".format(self._odometry.velocity,
                                                               self._vel_state.target_velocity,
                                                               self._vel_state.last_velocity))

        delta_v = self._lin_pid.update(self._odometry.velocity.linear)
        delta_w = self._ang_pid.update(self._odometry.velocity.angular)

        self._lin_pid.print(rospy.logdebug)
        self._ang_pid.print(rospy.logdebug)

        rospy.logdebug("TV - dv {:02.3f}, dw {:02.3f}".format(delta_v, delta_w))

        self._vel_state.target_velocity.linear += delta_v
        self._vel_state.target_velocity.angular += delta_w

        rospy.logdebug("TV Exit - O: {}, T: {}, L: {}".format(self._odometry.velocity,
                                                             self._vel_state.target_velocity,
                                                             self._vel_state.last_velocity))

    def _process_velocity(self):
        rospy.logdebug("PV Entry - T: {}, L: {}".format(self._vel_state.target_velocity, self._vel_state.last_velocity))

        self._safety_check()

        if self._velocity_has_changed():
            rospy.logdebug("Velocity Changed")
            self._limit_accel()

        else:
            rospy.logdebug("Velocity Unchanged")
            self._vel_state.target_velocity.linear = self._vel_state.last_velocity.linear
            self._vel_state.target_velocity.angular = self._vel_state.last_velocity.angular

        #self._lin_pid.set_target(self._vel_state.target_velocity.linear)
        #self._ang_pid.set_target(self._vel_state.target_velocity.angular)

        rospy.logdebug("PV Exit - T: {}, L: {}".format(self._vel_state.target_velocity, self._vel_state.last_velocity))

    def _update_speed(self):
        linear, angular = 0.0, 0.0
        if self._vel_state.target_velocity.linear != 0.0 or self._vel_state.target_velocity.angular != 0.0:
            linear, angular = self._vel_state.target_velocity.linear, self._vel_state.target_velocity.angular

        self._speedout_pub.publish(SpeedData(linear=linear, angular=angular))

    def start(self):
        rospy.loginfo("DriveNode starting ...")
        self._speedout_pub.publish(SpeedData(linear=0.0, angular=0.0))
        rospy.loginfo("DriveNode started")

    def loop(self):
        rospy.loginfo("DriveNode loop starting ...")
        while not rospy.is_shutdown():

            # Smooth velocity
            self._process_velocity()

            # Update Speed
            self._update_speed()

            # Track velocity
            #self._track_velocity()

            self._vel_state.last_velocity.linear = self._vel_state.target_velocity.linear
            self._vel_state.last_velocity.angular = self._vel_state.target_velocity.angular

            # Publish odometry
            self._publish_odom()

            rospy.loginfo("Linear(T/O): {:6.3f}/{:6.3f}, Angular(T/O): {:6.3f}/{:6.3f}".format(
                self._vel_state.target_velocity.linear,
                self._odometry.velocity.linear,
                self._vel_state.target_velocity.angular,
                self._odometry.velocity.angular))

            self._loop_rate.sleep()

        self.shutdown()
        rospy.loginfo("DriveNode loop exiting")

    def shutdown(self):
        rospy.loginfo("DriveNode shutdown")


if __name__ == "__main__":
    try:
        drivenode = DriveNode()
    except DriveNodeError as err:
        rospy.fatal("Unable to instantiate DriveNode - {}".format(err))
    else:
        try:
            drivenode.start()
            drivenode.loop()
        except rospy.ROSInterruptException as err:
            drivenode.shutdown()
            rospy.fatal("Interrupt Exception raised in DriveNode - {}".format(err))

# --- EOF ---

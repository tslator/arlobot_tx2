#!/usr/bin/env python

"""
A basic demo using odometry data to move the robot along a square trajectory
"""


from math import radians, sqrt, pow

import rospy
import tf
from geometry_msgs.msg import Twist, Point, Quaternion

from arlobot_ws.src.arlobot_tx2.common.transforms import quat_to_angle, normalize_angle


class NavSquare():
    def __init__(self):
        rospy.init_node('nav_square', anonymous=False)

        rospy.on_shutdown(self.shutdown)

        rate = 20

        r = rospy.Rate(rate)

        goal_distance = rospy.get_param("~goal_distance", 1.0)
        goal_angle = radians(rospy.get_param("~goal_angle", 90))
        linear_speed = rospy.get_param("~linear_speed", 0.3)
        angular_speed = rospy.get_param("~angular_speed", 0.2)
        angular_tolerance = radians(rospy.get_param("~angular_tolerance", 2))

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        self.base_frame = rospy.get_param("~base_frame", '/base_link')

        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        self.tf_listener = tf.TransformListener()

        rospy.sleep(2)

        self.odom_frame = '/odom'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
            rospy.signal_shutdown('tf Exception')

        position = Point()

        for i in range(4):
            rospy.loginfo("Starting ...")
            move_cmd = Twist()

            move_cmd.linear.x = linear_speed

            (position, rotation) = self.get_odom()
            rospy.loginfo("pos: {}, rot: {}".format(position, rotation))

            x_start = position.x
            y_start = position.y
            rospy.loginfo("x start: {}, y start: {}".format(x_start, y_start))

            distance = 0

            while distance < goal_distance and not rospy.is_shutdown():
                self.cmd_vel.publish(move_cmd)

                r.sleep()

                (position, rotation) = self.get_odom()
                rospy.loginfo("pos: {}, rot: {}".format(position, rotation))

                distance = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))
                rospy.loginfo("dist: {}".format(distance))

            rospy.loginfo("Stopping ...")

            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1.0)
            rospy.loginfo("Stopped")


            move_cmd.angular.z = angular_speed

            last_angle = rotation

            turn_angle = 0

            rospy.loginfo("Turning ...")

            while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
                self.cmd_vel.publish(move_cmd)

                r.sleep()

                (position, rotation) = self.get_odom()
                rospy.loginfo("pos: {}, rot: {}".format(position, rotation))

                delta_angle = normalize_angle(rotation - last_angle)

                turn_angle += delta_angle
                last_angle = rotation

            rospy.loginfo("Stopping ...")

            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1.0)
            rospy.loginfo("Stopped")

        self.cmd_vel.publish(Twist())

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def shutdown(self):
        rospy.loginfo("Stopping the robot ...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1.0)

if __name__ == "__main__":
    try:
        NavSquare()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navication terminated.")
#! /usr/bin/env python



# Standard Library
import math
from collections import namedtuple

# Third-Party
import rospy
from geometry_msgs.msg import Twist, Vector3
from arlobot_bringup.msg import HALPositionIn

# Project
#from basenode import BaseNode

class OutAndBack(object):

    def __init__(self):
        rospy.init_node("out_and_back", anonymous=False)

        rospy.on_shutdown(self.shutdown)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.sub = rospy.Subscriber('HALPositionIn', HALPositionIn, self._positionin_cb)

        linear_speed = 0.0081675
        angular_speed = 0.013
        timer = rospy.Rate(10)

        delta = 0
        t = Twist()
        t.linear.x=linear_speed
        t.angular.z=0.0

        while not rospy.is_shutdown() and delta < 5.0:

            # Move forward for 10 seconds
            self.pub.publish(t)
            timer.sleep()
            delta += 0.1
            rospy.loginfo("Delta: {}".format(delta))



        '''
        self.rate = 20.0
        #self.timer = rospy.Rate(self.rate)
        self.x = 0
        self.y = 0
        self._positionin_received = False

        Move = namedtuple('Move', 'linear angular timeout dist')

        forward = Move(linear=Vector3(x=linear_speed), angular=Vector3(), timeout=30, dist=1.0)
        stop = Move(linear=Vector3(), angular=Vector3(), timeout=1.0, dist=0.0)
        rotate = Move(linear=Vector3(), angular=Vector3(z=angular_speed), timeout=40, dist=math.pi*0.1969)

        moves = [
            forward,
            stop,
        ]
            rotate,
            stop,
            forward,
            stop,
            rotate,
            stop

        while not self._positionin_received:
            pass

        for m in moves:
            rospy.loginfo("Publishing {}".format(m))
            m.move(m)
            if rospy.is_shutdown():
                break
        rospy.loginfo("Timed out and back complete")
        '''

    def _positionin_cb(self, msg):
        self._positionin_received = True
        self.x = msg.x
        self.y = msg.y

    def _headingin_cb(self, msg):
        self.heading = msg.heading

    def ticks(self, dist, speed):
        try:
            return int((dist/speed)*self.rate)
        except ZeroDivisionError:
            return 0

    def do_angular_move(self, move):
        start_pos = self.heading
        curr_pos = start_pos
        end_time = rospy.Time.now() + rospy.Duration(move.timeout)

        while curr_pos - start_pos > move.dist and rospy.Time.now() < end_time:
            self.pub.publish(Twist(linear=move.linear, angular=move.angular))
            rospy.sleep(move.sleep)
            curr_pos = self.heading
            if rospy.is_shutdown():
                break

        rospy.loginfo("Dist: {}".format(curr_pos - start_pos))

    def do_linear_move(self, move):

        start_pos = math.sqrt(self.x**2 + self.y**2)
        curr_pos = start_pos
        end_time = rospy.Time.now() + rospy.Duration(move.timeout)
        while curr_pos - start_pos < move.dist and rospy.Time.now() < end_time:
            self.pub.publish(Twist(linear=move.linear, angular=move.angular))
            rospy.sleep(move.sleep)
            curr_pos = math.sqrt(self.x ** 2 + self.y ** 2)
            if rospy.is_shutdown():
                break

        rospy.loginfo("Dist: {}".format(curr_pos - start_pos))

    def shutdown(self):
        rospy.loginfo("Stopping the robot ...")
        self.pub.publish(Twist())
        rospy.sleep(1)



if __name__ == "__main__":
    try:
        OutAndBack()
    except rospy.ROSInterruptException:
        rospy.loginfo("Out-and-Back node terminated.")
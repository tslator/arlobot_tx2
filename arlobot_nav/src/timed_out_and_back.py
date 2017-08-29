"""

The purpose of this node is two fold:
  1. Implement a ROS node that performs the "Timed Out and Back" task
  2. Exposes a service that executes this task


* Timed Out and Back Requirements

1. Drive the robot forward for the specified distance at the specified speed
2. Rotate the robot 180 degrees at the specified speed
3. Drive the robot forward for the specified distance at the specified speed
4. Rotate the robot 180 degrees at the specified speed

Initialize Node
    - Set linear distance
    - Set linear speed
    - Set angular distance
    - Set angular speed
    - Set loop rate
    - Create cmd_vel publisher

Data Structure

    - list of moves
    - each move is a type (linear, angular), distance, speed
        - type and speed can be combined as a Twist

"""


# Standard Library
import math
from collections import namedtuple

# Third-Party
import rospy
from geometry_msgs.msg import Twist, Vector3

# Project
#from basenode import BaseNode

class OutAndBack(object):

    def __init__(self):
        rospy.init_node("out_and_back", anonymous=False)

        rospy.on_shutdown(self.shutdown)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        linear_distance = 1.0
        linear_speed = 0.2
        angular_distance = math.radians(180)
        angular_speed = 1.0

        rate = 10
        timer = rospy.Rate(rate)

        Move = namedtuple('Move', 'linear angular ticks timer')

        forward = Move(linear=Vector3(x=linear_speed), angular=Vector3(), ticks=self.ticks(linear_distance, linear_speed, rate), timer=timer)
        stop = Move(linear=Vector3(), angular=Vector3(), ticks=1, timer=timer)
        rotate = Move(linear=Vector3(), angular=Vector3(z=1.0), ticks=self.ticks(angular_distance, angular_speed, rate), timer=timer)

        moves = [
            forward,
            stop,
            rotate,
            stop,
            forward,
            stop,
            rotate,
            stop
        ]

        for m in moves:
            rospy.loginfo("Publishing {}".format(m))
            self.do_move(m)

    def ticks(self, dist, speed, rate):
        try:
            return int((dist/speed) * rate)
        except ZeroDivisionError:
            return 0

    def do_move(self, move):
        for t in range(move.ticks):
            self.pub.publish(Twist(linear=move.linear, angular=move.angular))
            move.timer.sleep()

    def shutdown(self):
        rospy.loginfo("Stopping the robot ...")
        self.pub.publish(Twist())
        rospy.sleep(1)



if __name__ == "__main__":
    try:
        OutAndBack()
    except rospy.ROSInterruptException:
        rospy.loginfo("Out-and-Back node terminated.")
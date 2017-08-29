#!/usr/bin/env python

"""
Republish the /robot_pose_ekf/odom_combined topic which is of type geometry_msgs/PoseWithCovarianceStamped as an
equivalent message of type nav_msgs/Odometry so we can view it in RViz
"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class OdomEKF(object):
    def __init__(self):
        rospy.init_node('odom_ekf', anonymous=False)

        self.ekf_pub = rospy.Publisher('output', Odometry, queue_size=5)

        rospy.wait_for_message('input', PoseWithCovarianceStamped)

        rospy.Subscriber('input', PoseWithCovarianceStamped, self._pub_ekf_odom)

        rospy.loginfo("Publishing combined odometry on /odom_ekf")

    def _pub_ekf_odom(self, msg):
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = '/odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose = msg.pose

        self.ekf_pub.publish(odom)

if __name__ == "__main__":
    try:
        OdomEKF()
        rospy.spin()
    except:
        pass
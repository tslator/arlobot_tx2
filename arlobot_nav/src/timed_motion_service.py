# The purpose of this node is to create a service that exposes three fundamental motions:
#   - LinearMove - either forward or backward but always in a straight line
#   - TurnMove - again, forward or backward and governed by linear/angular velocity
#   - StopMove - stops the robot

# With this service, clients can implement higher-level motions without needing to know
# the details.  With various services (or action servers), additional motion sensor can
# be used, EFK, lidar, ultrasonic, infrared, camera, etc.


import rospy



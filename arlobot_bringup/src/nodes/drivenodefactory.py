#!/usr/bin/env python


import rospy

from drivenode import SimulatedDriveNode, RealDriveNode


class DriveNodeFactoryError(Exception):
    pass


class DriveNodeFactory:
    """
    Provide factory to create the drive node
    """

    @classmethod
    def create_drive_node(cls, name, debug=False):

        simulated = rospy.get_param('/simulated', False)

        if simulated:
            return SimulatedDriveNode(name, debug)
        else:
            return RealDriveNode(name, debug)


if __name__ == "__main__":
    try:
        drivenode = DriveNodeFactory.create_drive_node("DriveNode")
    except DriveNodeFactoryError as err:
        rospy.fatal("Unable to instantiate DriveNode - {}".format(err))
    else:
        try:
            drivenode.start()
            drivenode.loop()
        except rospy.ROSInterruptException as err:
            drivenode.shutdown()
            rospy.fatal("Interrupt Exception raised in DriveNode - {}".format(err))

# --- EOF ---
#!/usr/bin/env python3

import rospy


def main():
    rospy.init_node("copy_robot_description_node", anonymous=True)
    rospy.sleep(1.0)
    mur_ns = rospy.get_param('~mur_ns')
    ur_prefix = rospy.get_param('~ur_prefix', "UR10")
    robot_description = rospy.get_param("/" + mur_ns + "/robot_description")
    rospy.set_param("/" + mur_ns + "/" + ur_prefix + "_l/robot_description", robot_description)
    rospy.set_param("/" + mur_ns + "/" + ur_prefix + "_r/robot_description", robot_description)


if __name__ == "__main__":
    main()
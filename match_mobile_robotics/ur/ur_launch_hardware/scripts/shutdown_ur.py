#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
import actionlib

if __name__=="__main__":
    rospy.init_node("shutdown_ur")
    pub=rospy.Publisher('/ur_hardware_interface/ur_hardware_interface/script_command', String)
    rospy.loginfo("Waiting for set_mode action server!")
    rospy.loginfo("Found set_mode action server!")
    pub.publish("powerdown()")
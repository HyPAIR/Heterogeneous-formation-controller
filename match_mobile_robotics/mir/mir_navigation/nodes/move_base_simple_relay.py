#!/usr/bin/env python3

from threading import local
import actionlib

import rospy
import mbf_msgs.msg as mbf_msgs
from dynamic_reconfigure.client import Client
from geometry_msgs.msg import PoseStamped


"""
move_base legacy relay node:
Relays old move_base actions to the new mbf move_base action, similar but with richer result and feedback.
We also relay the simple goal topic published by RViz, the make_plan service and dynamic reconfiguration
calls (note that some parameters have changed names; see http://wiki.ros.org/move_base_flex for details)
"""

# keep configured base local and global planners to send to MBF
global global_planner_name 
global_planner_name = None
global local_planner_name
local_planner_name = None


def simple_goal_cb(msg):
    mbf_mb_ac.send_goal(mbf_msgs.MoveBaseGoal(target_pose=msg,
                                              planner=global_planner_name,
                                              controller=local_planner_name))

if __name__ == '__main__':
    rospy.init_node("move_base")

    if rospy.has_param("~global_planner_name"):
        global_planner_name = rospy.get_param("~global_planner_name")
    
    if rospy.has_param("~local_planner_name"):
        local_planner_name = rospy.get_param("~local_planner_name")

    rospy.loginfo("move_base_simple_relay will command move_base_flex to use the following planner:")
    rospy.loginfo("Global planner: " + global_planner_name)
    rospy.loginfo("Local planner: " + local_planner_name)

    # move_base_flex get_path and move_base action clients
    mbf_mb_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
    mbf_mb_ac.wait_for_server(rospy.Duration(20))

    # move_base_flex dynamic reconfigure client
    mbf_drc = Client("move_base_flex", timeout=10)

    # move_base simple topic and action server
    mb_sg = rospy.Subscriber('move_base_simple/goal', PoseStamped, simple_goal_cb)

    rospy.spin()

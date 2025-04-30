#! /usr/bin/env python3

import rospy
import actionlib
from ur_dashboard_msgs.msg import SetModeAction,SetModeGoal
from std_srvs.srv import Trigger,TriggerRequest

def feedbackCallback(feedback):
    rospy.loginfo("Feedback:")
    rospy.loginfo(feedback)

def activeCallback():
    rospy.loginfo("Enabling ur!")


def doneCallback(state,result):
    if result.success: 
        rospy.loginfo("Done:")
        rospy.loginfo("State:")
        rospy.loginfo(state)
        rospy.loginfo("Result:")
        rospy.loginfo(result)
    else:
        rospy.logwarn("Done:")
        rospy.logwarn("State:")
        rospy.logwarn(state)
        rospy.logwarn("Result:")
        rospy.logwarn(result)


if __name__=="__main__":
    rospy.init_node("ur_enabler")
    client=actionlib.SimpleActionClient("ur_hardware_interface/set_mode",SetModeAction)
    client.wait_for_server()
    start_client=rospy.ServiceProxy("ur_hardware_interface/dashboard/play",Trigger)
    stop_client=rospy.ServiceProxy("ur_hardware_interface/dashboard/stop",Trigger)
    
    trigger=TriggerRequest()
    
    stop_client.call(trigger)
    rospy.sleep(2)
    goal_start=SetModeGoal()
    goal_start.target_robot_mode=7
    client.send_goal(goal_start,feedback_cb=feedbackCallback,done_cb=doneCallback,active_cb=activeCallback)
    client.wait_for_result()

    rospy.sleep(2)
    start_client.call(trigger)   
    
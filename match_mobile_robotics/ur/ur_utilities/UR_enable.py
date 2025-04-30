#!/usr/bin/env python3

import rospy
import actionlib
from ur_dashboard_msgs.msg import SetModeAction, SetModeGoal, RobotMode

class UR_enable():
    
    
    def __init__(self):
        self.set_mode_topic = rospy.get_param('~set_mode_topic', '/UR10_r/ur_hardware_interface/set_mode')
    
    def main(self):
        # initialize action server to enable the robot
        client = actionlib.SimpleActionClient(self.set_mode_topic, SetModeAction)
        client.wait_for_server()
        
        set_mode_goal = SetModeGoal()
        robot_mode = RobotMode()
        robot_mode.mode = 7

        set_mode_goal.target_robot_mode = robot_mode
        set_mode_goal.play_program = True        
        client.send_goal(set_mode_goal)
        
        # wait for the action server to finish
        client.wait_for_result()
        rospy.loginfo("Robot enabled")
    
if __name__ == '__main__':
    rospy.init_node('UR_enable')
    UR_enable = UR_enable().main()


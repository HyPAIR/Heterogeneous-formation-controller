#!/usr/bin/env python3
from __future__ import print_function
import sys
import rospy
from controller_manager_msgs.srv import SwitchController

def startup_client():
    tf_prefix = rospy.get_param('~tf_prefix',"")
    start_controllers = rospy.get_param('~start_controllers',"joint_group_vel_controller")
    stop_controllers = rospy.get_param('~stop_controllers',"twist_controller")
    service_topic = tf_prefix+ "/controller_manager/switch_controller"
    print(service_topic)
    rospy.wait_for_service(service_topic)
    print("service found")
    rospy.sleep(0.1)
    try:
        switch_service = rospy.ServiceProxy(service_topic, SwitchController)
              
        result = switch_service([start_controllers],[stop_controllers],1,True,1.0) 
        print(result)
             
        
        
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    startup_client()

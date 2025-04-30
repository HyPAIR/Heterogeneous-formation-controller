#!/usr/bin/env python3
from __future__ import print_function
from std_srvs.srv import Trigger, TriggerRequest
import sys
import rospy

def startup_client():
    mur_ns = rospy.get_param('~mur_ns',"")
    left_arm_group_name = rospy.get_param('~left_arm_group_name',"UR10_l")
    right_arm_group_name = rospy.get_param('~right_arm_group_name',"UR10_r")
    service_topics = [mur_ns+ left_arm_group_name + "/ur_hardware_interface/dashboard/", mur_ns+ right_arm_group_name + "/ur_hardware_interface/dashboard/"]
    
    try:

        for i in range(0,len(service_topics)):
            rospy.wait_for_service(service_topics[i]+"quit")
            print("service found")
            rospy.sleep(0.1)
            quit_service = rospy.ServiceProxy(service_topics[i] + 'quit', Trigger)
            connect_service = rospy.ServiceProxy(service_topics[i] + 'connect', Trigger)
            power_on_service = rospy.ServiceProxy(service_topics[i] + 'power_on', Trigger)
            brake_release_service = rospy.ServiceProxy(service_topics[i] + 'brake_release', Trigger)
            stop_service = rospy.ServiceProxy(service_topics[i] + 'stop', Trigger)
            play_service = rospy.ServiceProxy(service_topics[i] + 'play', Trigger)
            
            quit            = TriggerRequest()
            connect         = TriggerRequest()
            power_on        = TriggerRequest()
            brake_release   = TriggerRequest()
            stop            = TriggerRequest()
            play            = TriggerRequest()
            
            result = quit_service(quit) 
            #print(result)
            rospy.sleep(0.1)
            
            result = connect_service(connect) 
            #print(result)
            rospy.sleep(0.1)

            result = power_on_service(power_on) 
            #print(result)
            rospy.sleep(0.1)

            result = brake_release_service(brake_release) 
            #print(result)
            rospy.sleep(20.0)
            
            result = stop_service(stop) 
            #print(result)
            rospy.sleep(0.1)
            
            result = play_service(play) 
            #print(result)
        
        
        
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    startup_client()

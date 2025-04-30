#!/usr/bin/env python3
from __future__ import print_function
from rospy import service
from std_srvs.srv import Trigger, TriggerRequest
import sys
import rospy

def startup_client():
    tf_prefix = rospy.get_param('~tf_prefix',"")
    service_topic = tf_prefix+ "/ur_hardware_interface/dashboard/"
    print(service_topic)
    rospy.wait_for_service(service_topic+"quit")
    print("service found")
    rospy.sleep(0.1)
    try:
        quit_service = rospy.ServiceProxy(service_topic + 'quit', Trigger)
        connect_service = rospy.ServiceProxy(service_topic + 'connect', Trigger)
        stop_service = rospy.ServiceProxy(service_topic + 'stop', Trigger)
        play_service = rospy.ServiceProxy(service_topic + 'play', Trigger)
        
        quit    = TriggerRequest()
        connect = TriggerRequest()
        stop    = TriggerRequest()
        play    = TriggerRequest()
        
        result = quit_service(quit) 
        print(result)
        rospy.sleep(0.1)
        
        result = connect_service(connect) 
        print(result)
        rospy.sleep(0.1)
        
        result = stop_service(stop) 
        print(result)
        rospy.sleep(0.1)
        
        result = play_service(play) 
        print(result)
        
        
        
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    startup_client()

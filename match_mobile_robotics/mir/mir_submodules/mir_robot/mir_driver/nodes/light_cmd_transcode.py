#!/usr/bin/env python3

# this node subsribes to /light_cmd and creates a service call to /light_cmd_srv
# this is done becuase the mir_bridge will not pass service calls

import rospy
from mir_srvs.srv import LightCommand
import os
import mir_msgs.msg
from light_cmd_transcode_local import LightCmdTranscodeLocal
import socket
import json

class LightCmdTranscode():
    def __init__(self):
        local_master_uri = "http://roscore:11311"
        remote_master_uri = "http://192.168.12.20:11311"
        self.sub_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # wait for socket to be ready
        rospy.sleep(1.0)
        self.sub_socket.connect(('localhost', 8000))
        self.set_ros_master_uri(remote_master_uri)
        rospy.init_node('light_cmd_transcode_remote')
        rospy.wait_for_service('/light_srv')
        rospy.loginfo("LightCmdTranscode: Service /light_srv is available")
        self.light_cmd_srv = rospy.ServiceProxy('/light_srv', LightCommand)
        
        self.main()
        
    def main(self):
        rospy.on_shutdown(self.shutdown)
        rate = rospy.Rate(100) # 10hz
        while not rospy.is_shutdown():
            json_data = self.sub_socket.recv(1024).decode()
            data_dict = json.loads(json_data)
            data_list = [data_dict['effect'], data_dict['color1'], data_dict['color2'], data_dict['leds'],
                     data_dict['token'], data_dict['timeout'], data_dict['priority']]
            result = self.light_cmd_srv(*data_list)
            rate.sleep()
                    
    def set_ros_master_uri(self,remote_uri):
        os.environ["ROS_MASTER_URI"] = remote_uri

    def shutdown(self):
        self.sub_socket.shutdown()
        self.sub_socket.close()

if __name__ == '__main__':
    try:
        
        LightCmdTranscode()
    except rospy.ROSInterruptException:
        pass

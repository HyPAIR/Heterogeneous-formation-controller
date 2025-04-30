#!/usr/bin/env python3

import rospy
import os
import mir_msgs.msg
from mir_srvs.srv import LightCommand
import socket
import json

class LightCmdTranscodeLocal():
    def __init__(self):
        rospy.init_node('light_cmd_transcode_local', anonymous=True)
        rospy.on_shutdown(self.__on_shutdown)
        pub_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        pub_socket.bind(('localhost', 8000))
        pub_socket.listen(1)
        self.conn, addr = pub_socket.accept()
        rospy.Subscriber('new_light_cmd', mir_msgs.msg.LightCmd, self.callback)        
        rospy.spin()

    def callback(self,data):

        # convert data to json and send it
        data = {
            'effect': data.effect,
            'color1': data.color1,
            'color2': data.color2,
            'leds': data.leds,
            'token': data.token,
            'timeout': data.timeout,
            'priority': data.priority
        }
        json_data = json.dumps(data)        
        self.conn.sendall(json_data.encode())
        
    def __on_shutdown(self):
        rospy.loginfo("Shutting down light_cmd_transcode_local node")
        self.conn.shutdown(socket.SHUT_RDWR)
        self.conn.close()
        rospy.loginfo("Closed socket")

if __name__ == '__main__':
    try:
        LightCmdTranscodeLocal()
    except rospy.ROSInterruptException:
        pass
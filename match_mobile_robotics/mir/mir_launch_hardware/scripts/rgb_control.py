#!/usr/bin/env python3

# This node offers a number of services to control the RGB LEDs of the MIR robot.

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from mir_msgs.msg import LightCmd
from std_msgs.msg import String 
from mir_srvs.srv import LightCommand, ColorRGB, ColorRGBResponse


class RGBControl():
    
    def __init__(self):
        rospy.loginfo('RGB_Control: init')
        # create trigger service for rainbow effect
        rospy.Service('RGB_control/rainbow_start', Trigger, self.light_effect)
        rospy.Service('RGB_control/rainbow_stop', Trigger, self.light_effect)
        rospy.Service('RGB_control/solid_color', ColorRGB, self.light_effect)
        rospy.Service('RGB_control/match_color', Trigger, self.light_effect)
        self.light_cmd_pub = rospy.Publisher('new_light_cmd', LightCmd, queue_size=1)
        self.light_cmd = LightCmd()
        rospy.sleep(1) # wait for the mir to be ready
        self.default_effect()
        rospy.spin()


    def default_effect(self):
        self.light_cmd.color1 = "509600"
        self.light_cmd.color2 = "ffffff"
        self.light_cmd.priority = 1000
        self.light_cmd.leds = "all"
        self.light_cmd.effect = "solid"
        self.light_cmd_pub.publish(self.light_cmd)
        
    def light_effect(self, req):
        request_topic = req._connection_header['service']
        request = request_topic.split('/')[-1] 

        if request == 'rainbow_start':
            rospy.loginfo('rainbow_start')
            self.light_cmd.color1 = "ffffff"
            self.light_cmd.color2 = "ffffff"
            self.light_cmd.priority = 1000
            self.light_cmd.leds = "all"
            self.light_cmd.effect = "rainbow"
            self.light_cmd_pub.publish(self.light_cmd)
            return TriggerResponse(success=True, message='OK')
        elif request == 'rainbow_stop':
            rospy.loginfo('rainbow_stop')
            self.light_cmd.color1 = "ffff00"
            self.light_cmd.color2 = "ff0000"
            self.light_cmd.priority = 1000
            self.light_cmd.leds = "all"
            self.light_cmd.effect = "Wave"
            self.light_cmd_pub.publish(self.light_cmd)
            return TriggerResponse(success=True, message='OK')
        elif request == 'match_color':
            rospy.loginfo('match_color')
            self.light_cmd.color1 = "ffff00"
            self.light_cmd.color2 = "509600"
            self.light_cmd.priority = 1000
            self.light_cmd.leds = "all"
            self.light_cmd.effect = "solid"
            self.light_cmd_pub.publish(self.light_cmd)
            return TriggerResponse(success=True, message='OK')
        elif request == 'solid_color':
            rospy.loginfo('solid_color')
            colorR = hex(req.red)[2:]
            if len(colorR) == 1:
                colorR = "0" + colorR
            colorG = hex(req.green)[2:]
            if len(colorG) == 1:
                colorG = "0" + colorG
            colorB = hex(req.blue)[2:]
            if len(colorB) == 1:
                colorB = "0" + colorB
            color = colorR + colorG + colorB

            self.light_cmd.color1 = color
            self.light_cmd.color2 = "ffffff"
            self.light_cmd.priority = 1000
            self.light_cmd.leds = "all"
            self.light_cmd.effect = "solid"
            self.light_cmd_pub.publish(self.light_cmd)
            return True
        else:
            rospy.logwarn('Unknown service: ' + req._connection_header['service'])
            
        
            
        
        
    
    
if __name__ == '__main__':
    rospy.init_node('rgb_control')
    RGBControl()
    
    
    
    


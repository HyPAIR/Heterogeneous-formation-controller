#!/usr/bin/env python3

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from std_srvs.srv import Trigger, TriggerResponse

# the grasp server depends on the gazebo_ros_link_attacher plugin

class Grasp_server():

    def __init__(self):
        rospy.init_node('grasp_server_node')
        self.robot_name = rospy.get_param("~robot_name")
        self.gripper_link = rospy.get_param("~gripper_link")
        self.object_name = rospy.get_param("~object_name")
        self.object_link = rospy.get_param("~object_link")
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
        self.attach_srv.wait_for_service()
        rospy.Service('attach', Trigger, self.attach)
        rospy.spin()


    def attach(self,request):
        req = AttachRequest()
        req.model_name_1 = self.robot_name
        req.link_name_1 = self.gripper_link
        req.model_name_2 = self.object_name
        req.link_name_2 = self.object_link

        self.attach_srv.call(req)
        return TriggerResponse(
            success=True,
            message="Link attached"
        )
        

if __name__ == '__main__':
    Grasp_server()
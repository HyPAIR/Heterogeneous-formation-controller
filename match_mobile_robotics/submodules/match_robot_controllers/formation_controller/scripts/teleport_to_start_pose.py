#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist

from tf import transformations
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelStateResponse, GetModelState, GetModelStateRequest, GetModelStateResponse

class Teleport_to_start_pose():

    def __init__(self):
        self.target_pose = rospy.get_param("~target_pose", [5,1,0])
        self.set_model_service = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.robot_name = rospy.get_param("~robot_name", "mir")

        rospy.sleep(0.1)

        
    def teleport_robot(self):
        req = SetModelStateRequest()
        req.model_state.model_name = self.robot_name
        req.model_state.pose.position.x = self.target_pose[0]
        req.model_state.pose.position.y = self.target_pose[1]
        q = transformations.quaternion_from_euler(0, 0, self.target_pose[2])
        req.model_state.pose.orientation.x = q[0]
        req.model_state.pose.orientation.y = q[1]
        req.model_state.pose.orientation.z = q[2]
        req.model_state.pose.orientation.w = q[3]

        req.model_state.pose.position.z = 0
        self.set_model_service.call(req)
        rospy.sleep(0.1)

    


if __name__ == '__main__':
    try:
        rospy.init_node('move_to_start_pose')
        rospy.loginfo('move_to_start_pose node started')
        exe = Teleport_to_start_pose()
        exe.teleport_robot()
    except rospy.ROSInterruptException:
        pass
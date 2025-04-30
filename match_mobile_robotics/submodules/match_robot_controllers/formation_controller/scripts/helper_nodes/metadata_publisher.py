#! /usr/bin/env python3
import rospy
from formation_controller.msg import controller_metadata
from tf import transformations


class Metadata_publisher():
    def __init__(self):
        self.metadata_publisher = rospy.Publisher('controller_metadata', controller_metadata, queue_size=1)
        self.controller_metadata = controller_metadata()

        

    def publish_controller_metadata(self,**kwargs):
        if "target_pose" in kwargs:
            target_pose = kwargs["target_pose"]
            self.controller_metadata.target_pose_x = target_pose.position.x
            self.controller_metadata.target_pose_y = target_pose.position.y
            self.controller_metadata.target_pose_theta = target_pose.orientation.w
        if "error" in kwargs:
            error = kwargs["error"]
            self.controller_metadata.error_x = error[0]
            self.controller_metadata.error_y = error[1]
            self.controller_metadata.error_theta = error[2]
        if "leader_pose" in kwargs:
            leader_pose = kwargs["leader_pose"]
            self.controller_metadata.leader_pose_x = leader_pose.position.x
            self.controller_metadata.leader_pose_y = leader_pose.position.y
            self.controller_metadata.leader_pose_theta = transformations.euler_from_quaternion([leader_pose.orientation.x, leader_pose.orientation.y, leader_pose.orientation.z, leader_pose.orientation.w])[2]
        if "actual_pose" in kwargs:
            actual_pose = kwargs["actual_pose"]
            self.controller_metadata.actual_pose_x = actual_pose.position.x
            self.controller_metadata.actual_pose_y = actual_pose.position.y
            self.controller_metadata.actual_pose_theta = transformations.euler_from_quaternion([actual_pose.orientation.x, actual_pose.orientation.y, actual_pose.orientation.z, actual_pose.orientation.w])[2]
        if "target_velocity" in kwargs:
            target_velocity = kwargs["target_velocity"]
            self.controller_metadata.target_velocity_x = target_velocity.linear.x
            self.controller_metadata.target_velocity_y = target_velocity.linear.y
        if "robot_id" in kwargs:
            robot_id = kwargs["robot_id"]
            self.controller_metadata.robot_id = robot_id



        if "publish" in kwargs:
            publish = kwargs["publish"]
            if publish == True:
                self.controller_metadata.header.stamp = rospy.Time.now()
                self.metadata_publisher.publish(self.controller_metadata)



        
            


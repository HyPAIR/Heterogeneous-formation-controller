#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin, sqrt, atan2, inf
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class MoveToLeaderPose():

    def __init__(self):
        self.config()
        self.got_leader_pose = False
        self.reached_target = False
        self.mir_pose_topic = "/" + self.tf_prefix + "/mir_pose_stamped_simple"
        rospy.Subscriber(self.mir_pose_topic, PoseStamped, self.robot_pose_callback)
        rospy.Subscriber(self.leader_pose_topic, PoseWithCovarianceStamped, self.leader_pose_callback)
        self.move_base_client = actionlib.SimpleActionClient('/' + self.tf_prefix + '/move_base_flex/move_base', MoveBaseAction)
        print(self.tf_prefix + '/move_base_flex/move_base')
        self.move_base_client.wait_for_server()
        print("got server")
        rospy.sleep(1.0)
        if not self.got_leader_pose:
            rospy.logerr('Could not get leader pose. Shutting down node.')
            rospy.signal_shutdown('Could not get leader pose. Shutting down node.')
        else:
            rospy.loginfo('Got leader pose. Moving to target.')
            self.run()
            if self.reached_target:
                rospy.loginfo('Reached target. Shutting down node.')
                rospy.signal_shutdown('Reached target. Shutting down node.')
            else:
                rospy.logerr('Could not reach target. Shutting down node.')
                rospy.signal_shutdown('Could not reach target. Shutting down node.')
            

    def config(self):
        self.leader_pose_topic = rospy.get_param('~leader_pose_topic', '/leader_pose')
        self.tf_prefix = rospy.get_param('~tf_prefix', 'mir')
        self.relative_pose = rospy.get_param('~relative_pose', [0.0, 0.0, 0.0])
        self.goal_tolerance_lin = rospy.get_param('~goal_tolerance', 0.1)
        self.goal_tolerance_ang = rospy.get_param('~goal_tolerance', 0.05)

        
    def run(self):
        # apply transformation to leader pose
        self.robot_target_pose = PoseStamped()
        self.robot_target_pose.header.stamp = rospy.Time.now()
        self.robot_target_pose.header.frame_id = self.leader_pose.header.frame_id
        angle = euler_from_quaternion([self.leader_pose.pose.pose.orientation.x, self.leader_pose.pose.pose.orientation.y, self.leader_pose.pose.pose.orientation.z, self.leader_pose.pose.pose.orientation.w])
        self.robot_target_pose.pose.position.x = self.leader_pose.pose.pose.position.x + self.relative_pose[0]*cos(angle[2]) - self.relative_pose[1]*sin(angle[2])
        self.robot_target_pose.pose.position.y = self.leader_pose.pose.pose.position.y + self.relative_pose[0]*sin(angle[2]) + self.relative_pose[1]*cos(angle[2])
        leader_angle = angle[2] + self.relative_pose[2]
        self.robot_target_pose.pose.orientation.x, self.robot_target_pose.pose.orientation.y, self.robot_target_pose.pose.orientation.z, self.robot_target_pose.pose.orientation.w = quaternion_from_euler(0.0, 0.0, leader_angle)
        
        # send transformed leader pose to move_base_client
        goal = MoveBaseGoal()
        goal.target_pose = self.robot_target_pose
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        
        # check if leader pose is reached
        while not rospy.is_shutdown():
            if self.move_base_client.get_state() == 3:
                rospy.loginfo('target pose reached.')
                self.reached_target = True
                break
            else:
                rospy.sleep(0.1)


    def move_precise(self):
        # check if robot pose topic is available
        rospy.wait_for_message(self.mir_pose_topic, PoseStamped)
        
        distance = inf

        while distance > self.goal_tolerance_lin:
            # compute distance between robot and leader
            distance = sqrt((self.leader_pose.pose.pose.position.x - self.robot_pose.pose.position.x)**2 + (self.leader_pose.pose.pose.position.y - self.robot_pose.pose.position.y)**2)

            # compute angle between robot and leader
            angle = atan2(self.leader_pose.pose.pose.position.y - self.robot_pose.pose.position.y, self.leader_pose.pose.pose.position.x - self.robot_pose.pose.position.x)

            

        


    def leader_pose_callback(self, data):
        self.leader_pose = data
        self.got_leader_pose = True

    def robot_pose_callback(self, data):
        self.robot_pose = data



if __name__ == '__main__':
    rospy.init_node('move_to_leader_pose')
    MoveToLeaderPose()
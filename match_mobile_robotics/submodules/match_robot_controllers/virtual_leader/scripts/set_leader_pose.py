#! /usr/bin/env python3

# this node is used to set the virtual leader's pose relativ to a given robot

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin, pi

class SetLeaderPose():

    def __init__(self):
        self.config()
        self.got_robot_pose = False
        rospy.Subscriber(self.robot_pose_topic, PoseStamped, self.robot_pose_callback)
        self.leader_pose_pub = rospy.Publisher(self.leader_pose_topic, PoseStamped, queue_size=1)
        rospy.sleep(1.0)
        if not self.got_robot_pose:
            rospy.logerr('Could not get robot pose. Shutting down node.')
            rospy.signal_shutdown('Could not get robot pose. Shutting down node.')
        else:
            rospy.loginfo('Got robot pose. Shutting down node.')
            rospy.signal_shutdown('Got robot pose. Shutting down node.')

    def config(self):
        self.robot_pose_topic = rospy.get_param('~robot_pose_topic', '/robot_pose')
        self.leader_pose_topic = rospy.get_param('~leader_set_pose_topic', '/set_pose')
        self.relative_pose = rospy.get_param('~relative_pose', [0.0, 0.0, 0.0])

    def robot_pose_callback(self, data):
        robot_pose = data.pose
        self.got_robot_pose = True
        leader_pose = PoseStamped()
        leader_pose.header.stamp = rospy.Time.now()
        leader_pose.header.frame_id = data.header.frame_id

        # calculate leader pose
        angle = euler_from_quaternion([robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])
        leader_pose.pose.position.x = robot_pose.position.x + self.relative_pose[0]*cos(angle[2]) - self.relative_pose[1]*sin(angle[2])
        leader_pose.pose.position.y = robot_pose.position.y + self.relative_pose[0]*sin(angle[2]) + self.relative_pose[1]*cos(angle[2])

        # calculate leader orientation

        leader_angle = angle[2] + self.relative_pose[2]
        leader_pose.pose.orientation.x, leader_pose.pose.orientation.y, leader_pose.pose.orientation.z, leader_pose.pose.orientation.w = quaternion_from_euler(pi/2, 0.0, leader_angle)

        # publish leader pose
        self.leader_pose_pub.publish(leader_pose)
        rospy.sleep(0.1) # wait till pose is published and kill node


if __name__ == '__main__':
    rospy.init_node('set_leader_pose')
    SetLeaderPose()
#! /usr/bin/env python3

# this node is used to set the virtual follower's pose relativ to a given robot

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin, pi

class SetFollowerPose():

    def __init__(self):
        self.config()
        self.got_robot_pose = False
        rospy.Subscriber(self.leader_pose_topic, PoseStamped, self.robot_pose_callback)
        self.follower_pose_pub = rospy.Publisher(self.follower_set_pose_topic, PoseStamped, queue_size=1)
        rospy.sleep(1.0)
        if not self.got_robot_pose:
            rospy.logerr('Could not get robot pose. Shutting down node.')
            rospy.signal_shutdown('Could not get robot pose. Shutting down node.')
        else:
            rospy.loginfo('Got robot pose. Shutting down node.')
            rospy.signal_shutdown('Got robot pose. Shutting down node.')

    def config(self):
        self.leader_pose_topic = rospy.get_param('~leader_pose_topic', '/virtual_leader/leader_pose')
        self.follower_set_pose_topic = rospy.get_param('~follower_set_pose_topic', '/set_pose')
        self.relative_pose = rospy.get_param('~relative_pose', [0.0, 0.0, 0.0])

    def robot_pose_callback(self, data):
        robot_pose = data.pose
        self.got_robot_pose = True
        follower_pose = PoseStamped()
        follower_pose.header.stamp = rospy.Time.now()
        follower_pose.header.frame_id = data.header.frame_id

        # calculate follower pose
        angle = euler_from_quaternion([robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])
        follower_pose.pose.position.x = robot_pose.position.x + self.relative_pose[0]*cos(angle[2]) - self.relative_pose[1]*sin(angle[2])
        follower_pose.pose.position.y = robot_pose.position.y + self.relative_pose[0]*sin(angle[2]) + self.relative_pose[1]*cos(angle[2])

        # calculate follower orientation

        follower_angle = angle[2] + self.relative_pose[2]
        follower_pose.pose.orientation.x, follower_pose.pose.orientation.y, follower_pose.pose.orientation.z, follower_pose.pose.orientation.w = quaternion_from_euler(pi/2, 0.0, follower_angle)

        # publish follower pose
        self.follower_pose_pub.publish(follower_pose)
        rospy.sleep(0.1) # wait till pose is published and kill node


if __name__ == '__main__':
    rospy.init_node('set_follower_pose')
    SetFollowerPose()
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
from tf import transformations
from std_msgs.msg import Bool

class VirtualFollowerHelper():

    def __init__(self,virtual_follower_class):
        self.virtual_follower=virtual_follower_class
        self.load_param()
        self.virtual_follower.e_stop = False
        self.virtual_follower.time_old = rospy.get_time()
        self.virtual_follower.follower_pose = PoseStamped()
        self.virtual_follower.master_vel = Twist()
        self.virtual_follower.d_pose = [0,0,0]
        self.virtual_follower.d_pose_R = [0,0,0]
        self.virtual_follower.follower_orientation = 0.0
        self.virtual_follower.mrs_vel_subscriber_time = rospy.Time.now()

        rospy.Subscriber(self.set_pose_topic, PoseStamped, self.set_pose_cb)
        rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_cb)

        self.virtual_follower.pub        = rospy.Publisher(self.follower_pose_topic, PoseStamped, queue_size=10)
        self.virtual_follower.pub_vel    = rospy.Publisher(self.follower_vel_topic, Twist, queue_size=10)

    def load_param(self):
        self.virtual_follower.rate = rospy.get_param('~/rate', 100.0)
        self.virtual_follower.follower_name = rospy.get_param('~follower_name','follower')
        self.set_pose_topic = "/" + self.virtual_follower.follower_name + "/" + rospy.get_param('~set_pose_topic','set_pose')  
        rospy.set_param('~test_follower_pose_topic','follower_pose')
        self.follower_vel_topic = "/" + self.virtual_follower.follower_name + "/" + rospy.get_param('~follower_vel_topic','follower_vel') 
        self.follower_pose_topic = "/" + self.virtual_follower.follower_name + "/" + rospy.get_param('~follower_pose_topic','follower_pose')
        self.cmd_vel_topic = "/" + self.virtual_follower.follower_name + "/" + rospy.get_param('~cmd_vel_topic','cmd_vel')
        self.virtual_follower.cmd_vel_timeout = rospy.get_param('~cmd_vel_timeout','/cmd_vel_timeout')
        
    def set_pose_cb(self,data = PoseStamped()):
        self.virtual_follower.follower_pose.pose = data.pose
        orientation = transformations.euler_from_quaternion([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])
        self.virtual_follower.follower_orientation = orientation[2]
        
    def cmd_vel_cb(self,data):
        self.virtual_follower.master_vel = data
        self.virtual_follower.mrs_vel_subscriber_time = rospy.Time.now()


#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
from tf import transformations
from std_msgs.msg import Bool

class Virtual_leader_helper():

    def __init__(self,virtual_leader_class):
        self.virtual_leader=virtual_leader_class
        self.load_param()
        self.virtual_leader.e_stop = False
        self.virtual_leader.time_old = rospy.get_time()
        self.virtual_leader.leader_pose = PoseStamped()
        self.virtual_leader.master_vel = Twist()
        self.virtual_leader.d_pose = [0,0,0]
        self.virtual_leader.d_pose_R = [0,0,0]
        self.virtual_leader.leader_orientation = 0.0
        self.virtual_leader.mrs_vel_subscriber_time = rospy.Time.now()

        rospy.Subscriber(self.set_pose_topic, PoseStamped, self.set_pose_cb)
        rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_cb)

        self.virtual_leader.pub        = rospy.Publisher(self.leader_pose_topic, PoseStamped, queue_size=10)
        self.virtual_leader.pub_vel    = rospy.Publisher(self.leader_vel_topic, Twist, queue_size=10)

    def load_param(self):
        self.virtual_leader.rate = rospy.get_param('~/rate', 100.0)
        self.set_pose_topic = rospy.get_param('~set_pose_topic','set_pose')
        rospy.set_param('~test_leader_pose_topic','leader_pose')
        self.leader_vel_topic = rospy.get_param('~leader_vel_topic','leader_vel')
        self.leader_pose_topic = rospy.get_param('~leader_pose_topic','leader_pose')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic','cmd_vel')
        self.virtual_leader.cmd_vel_timeout = rospy.get_param('~cmd_vel_timeout','/cmd_vel_timeout')

    def set_pose_cb(self,data = PoseStamped()):
        self.virtual_leader.leader_pose.pose = data.pose
        orientation = transformations.euler_from_quaternion([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])
        self.virtual_leader.leader_orientation = orientation[2]
        
    def cmd_vel_cb(self,data):
        self.virtual_leader.master_vel = data
        self.virtual_leader.mrs_vel_subscriber_time = rospy.Time.now()


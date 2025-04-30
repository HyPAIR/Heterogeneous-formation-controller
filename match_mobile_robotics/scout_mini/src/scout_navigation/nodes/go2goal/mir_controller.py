#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry   # the "nav_msgs.msg" should be changed laut the command: $rosmsg show Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

x = 0.0
y = 0.0 
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x    # the "pose.pose" should be changed laut the command: $rosmsg show Odometry
    y = msg.pose.pose.position.y    # the "pose.pose" should be changed laut the command: $rosmsg show Odometry

    rot_q = msg.pose.pose.orientation   # the "pose.pose" should be changed laut the command: $rosmsg show Odometry
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("speed_controller")

sub = rospy.Subscriber("/odom_comb", Odometry, newOdom)  # the "/odom_enc" should be changed laut the command: $rostopic list
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)

goal = Point()  # the "Point" should be changed laut the command: $rosmsg show Odometry
goal.x = input("Set your x goal: ")
goal.y = input("Set your y goal: ")

while not rospy.is_shutdown():
    inc_x = goal.x -x
    inc_y = goal.y -y

    angle_to_goal = atan2(inc_y, inc_x)
    try:
        if abs(angle_to_goal - theta) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        else:
            speed.linear.x = 0.5
            speed.angular.z = 0.0

        pub.publish(speed)
        r.sleep()
    except rospy.ROSInterruptException:
        pass

     
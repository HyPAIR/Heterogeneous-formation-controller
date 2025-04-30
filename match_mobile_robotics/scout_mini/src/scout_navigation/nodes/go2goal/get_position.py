#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Imu
import numpy as np
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionGoal
import actionlib
#from actionlib_msgs.msg import *

 
def odom_callback(msg):
    print ("------------------------------------------------")
    print ("pose x = " + str(msg.pose.pose.position.x))
    print ("pose y = " + str(msg.pose.pose.position.y))
    
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    xy.append(x)
    xy.append(y)
    print(xy)

    ac = MoveBaseActionResult()
    GoalStatus = ac.status.SUCCEEDED
    print(GoalStatus)

    if GoalStatus != 3:
        i=0
        l = len(xy)
        path_legth = 0.0
        while i<= l-4:
          path_legth += np.hypot(xy[i+2]-xy[i], xy[i+3]-xy[i+1])
          i += 2
          print(i)


# def goal_callback(msg):
#     print ("------------------------------------------------")
#     print ("goal_x = " + str(msg.goal.target_pose.pose.position.x))
    #goal_x = msg.pose.position.x
    #print(goal_x)

    # ac = MoveBaseActionResult()
    # GoalStatus = ac.status.SUCCEEDED
    # print(GoalStatus)

    #goal = PoseStamped()
    #goal_x = goal.pose.position.x
    #print(goal_x)

# def imu_callback(msg):
#      print ("------------------------------------------------")
#      print ("veloc angular z = " + str(msg.angular_velocity.z))
#      print ("veloc angular y = " + str(msg.angular_velocity.y))
#      print ("aceleracion linear x = " + str(msg.linear_acceleration.x))
#      print ("aceleracion linear y = " + str(msg.linear_acceleration.y))

#def twist_callback(msg):
    # move = Twist()
    #print ("------------------------------------------------")
    #print ("velocidad linear x = " + str(msg.linear.x)) 
    #print ("velocidad linear x = " + str(msg.linear.y)) 
    #print ("velocidad angular z = " + str (move.angular.z))


xy = []
#i=0
#x=0
#y=0


#xy.append(x)
#xy.append(y)

# xy['x'] = x
# xy['y'] = y

rospy.init_node('mir_monitor') 
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) 
sub_odom = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, odom_callback)
#sub_goal = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, goal_callback)
#sub_imu = rospy.Subscriber('/imu_data', Imu, imu_callback)
#sub_vel = rospy.Subscriber('/cmd_vel', Twist, vel_callback) 
#sub_vel = rospy.Subscriber('/cmd_vel', Twist, twist_callback) 
rate = rospy.Rate(0.5)
#print(xy)
#print(xy)
#print(data[0])
#print(data[1])

 
while not rospy.is_shutdown():
    move = Twist()
    pub.publish(move)
    rate.sleep() 

# ac = MoveBaseActionResult()
# GoalStatus = ac.status.SUCCEEDED
# print(GoalStatus)
# if(GoalStatus == 4):
#      l = len(xy)
#      while i<= l-4:
#          path_legth += np.hypot(xy[i+2]-xy[i], xy[i+3]-xy[i+1])
#          i += 2
#          print(i)
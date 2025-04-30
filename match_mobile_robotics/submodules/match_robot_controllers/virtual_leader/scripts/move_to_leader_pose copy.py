#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from tf import transformations
import math

class Move_to_start_pose():

    def __init__(self):
        self.target_vel = Twist()
        self.KP = rospy.get_param("~KP", 0.5)
        self.angle_tolerance = rospy.get_param("~angle_tolerance", 0.02)
        self.control_rate = rospy.get_param("~control_rate", 50)
        self.target_pose = rospy.get_param("~target_pose", [2,0,0])
        self.linear_vel_limit = 0.08
        self.angular_vel_limit = 0.2
        self.linear_tolerance = 0.15
        
        
        self.move_base_simple_goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/mur620c/mir/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/mur620c/mir/mir_pose_simple', Pose, self.mir_pose_callback)
        rospy.sleep(0.1)


    def run(self):
        # wait for pose
        rospy.loginfo('Waiting for first message from /mur620c/mir/mir_pose_simple...')
        rospy.wait_for_message('/mur620c/mir/mir_pose_simple', Pose)
        
        
        # send the robot to the target pose using move base action server
                    #print(client.get_result())
            # turn the robot to face the target pose
            
        for i in range(0,3):
            # check if the robot is already in position
            if abs(self.target_pose[1] - self.mir_pose.position.y) < self.linear_tolerance / 2 and abs(self.target_pose[0] - self.mir_pose.position.x) < self.linear_tolerance / 2:
                break
            self.turn_to_target_pose()
            self.move_to_target_pose()
            
            self.linear_tolerance *= 0.5
            self.angle_tolerance *= 0.5

        self.turn_to_target_orientation()

    def turn_to_target_pose(self):
        
        # set the control rate
        rate = rospy.Rate(self.control_rate)
        
        while not rospy.is_shutdown():
            # compute the target theta
            target_theta = math.atan2(self.target_pose[1] - self.mir_pose.position.y, self.target_pose[0] - self.mir_pose.position.x)
            
            # compute the difference between the current and target theta
            theta_diff = target_theta - self.current_theta

            # turn the robot the shorter way
            # if theta_diff > math.pi * 0.5 and theta_diff < math.pi:
            #     theta_diff -= math.pi
            # elif theta_diff < -math.pi * 0.5 and theta_diff > -math.pi:
            #     theta_diff += math.pi

            
            # make sure the difference is in the range [-pi, pi]
            if theta_diff > math.pi:
                theta_diff -= 2 * math.pi
            elif theta_diff < -math.pi:
                theta_diff += 2 * math.pi

            # compute the target velocity
            self.target_vel.angular.z = 0.3 * theta_diff
            
            # limit angular velocity
            if abs(self.target_vel.angular.z) > self.angular_vel_limit:
                self.target_vel.angular.z = self.angular_vel_limit * self.target_vel.angular.z / abs(self.target_vel.angular.z)

            # publish the target velocity
            self.cmd_vel_pub.publish(self.target_vel)

            # check if robot is aligned with the target pose
            if abs(theta_diff) < self.angle_tolerance:
                break

            # sleep
            rate.sleep()

        # stop the robot
        self.target_vel.angular.z = 0
        self.cmd_vel_pub.publish(self.target_vel)
        rospy.loginfo('Robot turned to the target pose')


    def turn_to_target_orientation(self):

        # set the control rate
        rate = rospy.Rate(self.control_rate)

        while abs(self.current_theta - self.target_pose[2]) > self.angle_tolerance and not rospy.is_shutdown():
            # compute the target theta
            target_theta = self.target_pose[2]

            # compute the difference between the current and target theta
            theta_diff = target_theta - self.current_theta
            
            # make sure the difference is in the range [-pi, pi]
            if theta_diff > math.pi:
                theta_diff -= 2 * math.pi
            elif theta_diff < -math.pi:
                theta_diff += 2 * math.pi

            # compute the target velocity
            self.target_vel.angular.z = self.KP * theta_diff
            
            # limit angular velocity
            if abs(self.target_vel.angular.z) > self.angular_vel_limit:
                self.target_vel.angular.z = self.angular_vel_limit * self.target_vel.angular.z / abs(self.target_vel.angular.z)
            
            # publish the target velocity
            self.cmd_vel_pub.publish(self.target_vel)
            
            print('current_theta: ', self.current_theta)

            # sleep
            rate.sleep()

        # stop the robot
        self.target_vel.angular.z = 0
        self.cmd_vel_pub.publish(self.target_vel)
        rospy.loginfo('Robot turned to the target pose')
        
    def move_to_target_pose(self):
        rate = rospy.Rate(self.control_rate)
        
        while not rospy.is_shutdown():
            
            linear_error = math.sqrt((self.target_pose[0] - self.mir_pose.position.x)**2 + (self.target_pose[1] - self.mir_pose.position.y)**2)

            # compute linear velocity
            vel_lin = 0.4 * linear_error
        
            # limit linear velocity
            if abs(vel_lin) > self.linear_vel_limit:
                vel_lin = self.linear_vel_limit * vel_lin / abs(vel_lin)
                
            self.target_vel.linear.x = vel_lin
            self.target_vel.angular.z = 0.0
            
            self.cmd_vel_pub.publish(self.target_vel)
            
            if linear_error < self.linear_tolerance:
                break
        
            rate.sleep()
            
        # stop the robot
        self.target_vel.linear.x = 0
        self.cmd_vel_pub.publish(self.target_vel)
        rospy.loginfo('Robot moved to the target pose')
        

    def mir_pose_callback(self, msg):
        self.mir_pose = msg
        # compute current theta
        self.current_theta = transformations.euler_from_quaternion([self.mir_pose.orientation.x, self.mir_pose.orientation.y, self.mir_pose.orientation.z, self.mir_pose.orientation.w])[2]





if __name__ == '__main__':
    try:
        rospy.init_node('move_to_start_pose')
        rospy.loginfo('move_to_start_pose node started')
        exe = Move_to_start_pose()
        exe.run()
    except rospy.ROSInterruptException:
        pass
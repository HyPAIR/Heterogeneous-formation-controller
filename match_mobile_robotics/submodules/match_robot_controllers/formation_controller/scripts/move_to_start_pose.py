#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from tf import transformations
import actionlib
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
import math

class Move_to_start_pose():

    def __init__(self):
        self.target_vel = Twist()
        self.KP = rospy.get_param("~KP", 0.5)
        self.angle_tolerance = rospy.get_param("~angle_tolerance", 0.1)
        self.control_rate = rospy.get_param("~control_rate", 50)
        self.target_pose = rospy.get_param("~target_pose", [5,1,0])
        
        self.move_base_simple_goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('mir_pose_simple', Pose, self.mir_pose_callback)
        rospy.sleep(0.1)


    def run(self):
        # send the robot to the target pose using move base action server
        client = actionlib.SimpleActionClient('move_base_flex/move_base', MoveBaseAction)
        client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = self.target_pose[0]
        goal.target_pose.pose.position.y = self.target_pose[1]
        goal.target_pose.pose.position.z = 0
        q = transformations.quaternion_from_euler(0, 0, self.target_pose[2])
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        client.send_goal(goal)

        # wait for the action server to finish performing the action
        wait = client.wait_for_result()

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            #print(client.get_result())
            # turn the robot to face the target pose
            self.turn_to_target_pose()

    def turn_to_target_pose(self):

        # set the control rate
        rate = rospy.Rate(self.control_rate)

        while abs(self.current_theta - self.target_pose[2]) > self.angle_tolerance:
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

            # publish the target velocity
            self.cmd_vel_pub.publish(self.target_vel)

            # sleep
            rate.sleep()

        # stop the robot
        self.target_vel.angular.z = 0
        self.cmd_vel_pub.publish(self.target_vel)
        rospy.loginfo('Robot turned to the target pose')

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
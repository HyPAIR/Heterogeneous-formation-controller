#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from math import sin, cos, pi
import numpy as np
from geometry_msgs.msg import Pose
import tf

class Direct_kinematics_publisher():

    def __init__(self):
        # Initialize the node
        rospy.init_node('direct_kinematics_publisher', anonymous=True)
        self.load_params()
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.DH_params = [[0,0,0.1807,pi/2],[0,-0.6127,0,0],[0,-0.57155,0,0],[0,0,0.17415,pi/2],[0,0,0.11985,-pi/2],[0,0,0.11655,0]]
        self.q = [0,0,0,0,0,0]
        self.tcp_pose = Pose()
        self.pose_pub = rospy.Publisher("tcp_pose", Pose, queue_size=10)
        rospy.Subscriber("joint_states", JointState, self.joint_state_callback)
        rospy.spin()


    def joint_state_callback(self,JointState):
        for i in range(0,6):
            for idx in range(0,len(JointState.name)):
                if JointState.name[idx] == self.ur_prefix + self.joint_names[i]:
                    self.q[i] = JointState.position[idx]
        T = self.compute_direct_kinematics()
        self.tcp_pose.position.x = -T[0][3] # x is pointing backwards
        self.tcp_pose.position.y = -T[1][3] # y is pointing to the left
        self.tcp_pose.position.z = T[2][3]
        q = tf.transformations.quaternion_from_matrix(T)
        self.tcp_pose.orientation.x = q[0]
        self.tcp_pose.orientation.y = q[1]
        self.tcp_pose.orientation.z = q[2]
        self.tcp_pose.orientation.w = q[3]
        self.pose_pub.publish(self.tcp_pose)


    def compute_DH_matrix(self,theta,d,a,alpha):
        return [[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
                [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                [0, sin(alpha), cos(alpha), d],
                [0, 0, 0, 1]]

    def compute_direct_kinematics(self):
        T = [[1,0,0,0],
             [0,1,0,0],
             [0,0,1,0],
             [0,0,0,1]]
        for i in range(0,6):
            T = np.dot(T,self.compute_DH_matrix(self.q[i] + self.delta_theta[i], self.DH_params[i][2] + self.delta_d[i] 
            ,self.DH_params[i][1] + self.delta_a[i], self.DH_params[i][3] + self.delta_alpha[i]))
        return T

    def load_params(self):
        self.ur_prefix = rospy.get_param('~ur_prefix', 'ur')
        self.delta_theta = rospy.get_param('~delta_theta', [0,0,0,0,0,0])
        self.delta_a = rospy.get_param('~delta_a', [0,0,0,0,0,0])
        self.delta_d = rospy.get_param('~delta_d', [0,0,0,0,0,0])
        self.delta_alpha = rospy.get_param('~delta_alpha', [0,0,0,0,0,0])

if __name__ == '__main__':
    try:
        Direct_kinematics_publisher()
    except rospy.ROSInterruptException:
        pass
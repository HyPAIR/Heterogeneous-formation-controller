#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from math import sin, cos, pi, sqrt
import numpy as np
from geometry_msgs.msg import Pose
from copy import deepcopy
import tf
from std_msgs.msg import Float64MultiArray

from ur_dual_arm_collision_dist import DualArmDistance

class Dual_arm_collision_avoidance():

    def __init__(self):
        self.load_parameters()
        self.inital_run_l = True
        self.inital_run_r = True
        self.collision = True
        self.near_collision = True
        self.joint_pose_l = Pose()
        self.joint_pose_r = Pose()
        self.latest_command_l = Float64MultiArray()
        self.latest_command_l.data = [0,0,0,0,0,0]
        self.latest_command_r = Float64MultiArray()
        self.latest_command_r.data = [0,0,0,0,0,0]
        self.joints_state_index_l = [0,1,2,3,4,5]
        self.joints_state_index_r = [0,1,2,3,4,5]
        self.q_l = [0,0,0,0,0,0]
        self.q_r = [0,0,0,0,0,0]
        self.DH_params = [[0,0,0.1807,pi/2],[0,-0.6127,0,0],[0,-0.57155,0,0],[0,0,0.17415,pi/2],[0,0,0.11985,-pi/2],[0,0,0.11655,0]]
        self.tf_listener = tf.TransformListener()
        self.command_repub_l = rospy.Publisher(self.controller_name_l + "/unsafe/command", Float64MultiArray, queue_size=1)
        self.command_repub_r = rospy.Publisher(self.controller_name_r + "/unsafe/command", Float64MultiArray, queue_size=1)
        rospy.Subscriber(self.joint_states_topic, JointState, self.joint_states_callback_l)
        rospy.Subscriber(self.joint_states_topic, JointState, self.joint_states_callback_r)
        rospy.Subscriber(self.controller_name_l + "/safe/command", Float64MultiArray, self.controller_callback_l)
        rospy.Subscriber(self.controller_name_r + "/safe/command", Float64MultiArray, self.controller_callback_r)
        rospy.sleep(1)

        self.dual_arm_dist = DualArmDistance(faster=False)

        self.run()

    def run(self):


        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            d_min = self.dual_arm_dist.calc_min_dist(self.q_l, self.q_r)

            if d_min < self.safety_dist_threshold:
                rospy.logerr_once("Collision detected!")
                self.collision = True
                self.stop_controllers()
            elif d_min < self.warning_dist_threshold:
                self.near_collision = True
                self.collision = False
                self.slow_controllers()
            else:
                self.collision = False
                self.near_collision = False                

            rate.sleep()

    def load_parameters(self):
        self.joint_states_topic = rospy.get_param('~joint_states_topic')
        self.joint_names = rospy.get_param('~joint_names')
        self.controller_name_l = rospy.get_param('~controller_name_l')
        self.controller_name_r = rospy.get_param('~controller_name_r')
        self.rate = rospy.get_param('~rate')
        self.ur_prefix_l = rospy.get_param('~ur_prefix_l')
        self.ur_prefix_r = rospy.get_param('~ur_prefix_r')
        self.tf_prefix = rospy.get_param('~tf_prefix')
        self.collision_objects_per_link = rospy.get_param('~collision_objects_per_link')
        self.safety_dist_threshold = rospy.get_param('~safety_dist_threshold')
        self.warning_dist_threshold = rospy.get_param('~warning_dist_threshold')

    def sort_joint_states_l(self, JointState):
        for i in range(0,6):
            for idx in range(0,len(JointState.name)):
                if JointState.name[idx] == self.ur_prefix_l + self.joint_names[i]:
                    self.joints_state_index_l[i] = idx

    def sort_joint_states_r(self, JointState):
        for i in range(0,6):
            for idx in range(0,len(JointState.name)):
                if JointState.name[idx] == self.ur_prefix_r + self.joint_names[i]:
                    self.joints_state_index_r[i] = idx
    def joint_states_callback_l(self, JointState):
        if self.inital_run_l:
            self.sort_joint_states_l(JointState)
            self.inital_run_l = False
        for i in range(0,6):
            self.q_l[i] = JointState.position[self.joints_state_index_l[i]]

    def joint_states_callback_r(self, JointState):
        if self.inital_run_r:
            self.sort_joint_states_r(JointState)
            self.inital_run_r = False
        for i in range(0,6):
            self.q_r[i] = JointState.position[self.joints_state_index_r[i]]

    def controller_callback_l(self, command):
        self.latest_command_l = command
        if self.collision == True:
            command.data = [0,0,0,0,0,0]
        elif self.near_collision == True:
            command.data = tuple(0.5 * elem for elem in command.data) # reduce speed by 50%
        else:
            pass
        self.command_repub_l.publish(command)

    def controller_callback_r(self, command):
        self.latest_command_r = command
        if self.collision == True:
            command.data = [0,0,0,0,0,0]
        elif self.near_collision == True:
            for i in range(0,len(command.data)):
                command.data = tuple(0.5 * elem for elem in command.data) # reduce speed by 50%
        else:
            pass
        self.command_repub_r.publish(command)
        

    def slow_controllers(self):
        if self.latest_command_l.data != None:
            self.latest_command_l.data = self.multipy_tuple(self.latest_command_l.data, 0.9)
            self.command_repub_l.publish(self.latest_command_l)
        if self.latest_command_r.data != [0,0,0,0,0,0]:
            self.latest_command_r.data = self.multipy_tuple(self.latest_command_r.data, 0.9) # reduce speed by 50%
            self.command_repub_r.publish(self.latest_command_r)

    def stop_controllers(self):
        stop_command = Float64MultiArray()
        stop_command.data = [0,0,0,0,0,0]
        self.command_repub_l.publish(stop_command)
        self.command_repub_r.publish(stop_command)

    def multipy_tuple(self, tup1, factor):
        data = list(tup1)
        return tuple(factor * elem for elem in data)
        
        


if __name__ == '__main__':
    rospy.init_node('dual_arm_collision_avoidance')
    dual_arm_collision_avoidance = Dual_arm_collision_avoidance()
    rospy.spin()
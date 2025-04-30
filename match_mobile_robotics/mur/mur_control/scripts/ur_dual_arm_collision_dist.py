#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from math import sin, cos, pi, sqrt
import numpy as np
from geometry_msgs.msg import Pose
from copy import deepcopy
import tf
from std_msgs.msg import Float64MultiArray
from typing import List

class DualArmDistance():

    def __init__(self, faster=True):
        """Calculates the minimal distance between two arms

        Args:
            faster (bool, optional): For faster calculation. Only takes joint 3+4 into account. Defaults to True.
        """
        self.load_parameters()
        self.DH_params = np.array([[0,0,0.1807,pi/2],[0,-0.6127,0,0],[0,-0.57155,0,0],[0,0,0.17415,pi/2],[0,0,0.11985,-pi/2],[0,0,0.11655,0]])
        self.num_calculated_transformations = 6
        if self.collision_objects_per_link > 1:
            self.skip_link_poses = False
        else:
            self.skip_link_poses = True
        self.faster = faster
        if faster:
            self.num_calculated_transformations = 4
            rospy.logerr("faster mode not implemented yet: DH directly to joint3 (and no extra points on links?)")
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1)

        # get trafos from mobile base to ur base
        self.trans_l = self.trans_r = []
        while self.trans_l == [] or self.trans_r == []:
            try:
                rospy.loginfo("Theoretically it would be sufficient to look up the transform from ur_l/base to ur_r/base in the future. But like this it can be easily extended for the mobile base as well")
                rospy.loginfo(f"looking for transformation from {self.tf_prefix +'/base_link'} to {self.tf_prefix + '/' + self.ur_prefix_l + 'base_link'}")
                self.trans_l = np.array(self.tf_listener.lookupTransform(self.tf_prefix +'/base_link', self.tf_prefix + "/" + self.ur_prefix_l + 'base_link', rospy.Time(0))[0])
                self.trans_r = np.array(self.tf_listener.lookupTransform(self.tf_prefix +'/base_link', self.tf_prefix + "/" + self.ur_prefix_r + 'base_link', rospy.Time(0))[0])
                rot_l = np.array(self.tf_listener.lookupTransform(self.tf_prefix + "/" + self.ur_prefix_l + 'base_link', self.tf_prefix +'/base_link', rospy.Time(0))[1])
                rot_r = np.array(self.tf_listener.lookupTransform(self.tf_prefix + "/" + self.ur_prefix_r + 'base_link', self.tf_prefix +'/base_link', rospy.Time(0))[1])
                self.T_l_0 = np.eye(4)
                self.T_r_0 = np.eye(4)
                self.T_l_0[0:3,0:3] = tf.transformations.quaternion_matrix(rot_l)[0:3,0:3]
                self.T_r_0[0:3,0:3] = tf.transformations.quaternion_matrix(rot_r)[0:3,0:3]
                rospy.loginfo(f"Init Rotation of left arm: {self.T_l_0[0:3,0:3]}")
                rospy.loginfo(f"Init Rotation of right arm: {self.T_r_0[0:3,0:3]}")
                break
            except:
                rospy.sleep(1.0)
                rospy.logerr_throttle(5,"waiting for tf to come up")
        rospy.loginfo("dual_arm_distance initialized")
        
    def calc_dist(self, q_l: List, q_r: List) -> float:
            # reset pose lists every run
            self.joint_pose_list_l = []
            self.joint_pose_list_r = []

            T_l = deepcopy(self.T_l_0)
            T_r = deepcopy(self.T_r_0)
            # compute pose of every joint
            for i in range(0,self.num_calculated_transformations):
                T_l = T_l.dot(self.compute_DH_matrix(q_l[i] , self.DH_params[i][2]  
                ,self.DH_params[i][1] , self.DH_params[i][3]))
                # p = self.trans_l[0]+T_l[0:3,3]
                x = T_l[0,3] + self.trans_l[0]
                y = T_l[1,3] + self.trans_l[1]
                z = T_l[2,3] + self.trans_l[2]
                self.joint_pose_list_l.append(np.array([x,y,z]))

                T_r = T_r.dot(self.compute_DH_matrix(q_r[i] , self.DH_params[i][2] 
                ,self.DH_params[i][1] , self.DH_params[i][3]))
                x = T_r[0][3] + self.trans_r[0] # right arm is rotated 180° around z
                y = T_r[1][3] + self.trans_r[1] # right arm is rotated 180° around z
                z = T_r[2][3] + self.trans_r[2]
                self.joint_pose_list_r.append(np.array([x,y,z]))

            # generate link poses (TODO: why only range(0,2)? and not range(5)?)
            if not self.skip_link_poses:
                for i in range(0,self.num_calculated_transformations - 1):
                    joint0_pose_l = self.joint_pose_list_l[i]
                    joint0_pose_r = self.joint_pose_list_r[i]
                    joint1_pose_l = self.joint_pose_list_l[i+1]
                    joint1_pose_r = self.joint_pose_list_r[i+1]
                    
                    direction_l = joint1_pose_l - joint0_pose_l
                    direction_r = joint1_pose_r - joint0_pose_r
                    step_len = 1 / (self.collision_objects_per_link+1)
                    for _ in range(0,self.collision_objects_per_link):
                        joint0_pose_l = joint0_pose_l + direction_l * step_len
                        joint0_pose_r = joint0_pose_r + direction_r * step_len
                        self.joint_pose_list_l.append(joint0_pose_l)
                        self.joint_pose_list_r.append(joint0_pose_r)
                    
            pose_matrix_l = np.array(self.joint_pose_list_l)
            pose_matrix_r = np.array(self.joint_pose_list_r)

            # dist = np.linalg.norm(pose_matrix_l - pose_matrix_r, axis=1) #TODO: false!!! Matrix would be ok
            # subtract every pose of one arm from every pose of the other arm:
            dist = np.linalg.norm(pose_matrix_l[:,None] - pose_matrix_r, axis=2).reshape(-1)
            # rospy.logdebug(f"{dist=}")
            return dist
    
    def calc_min_dist(self, q_l: List, q_r: List) -> float:
        dist = self.calc_dist(q_l, q_r)
        dist_min = np.min(dist)
        # rospy.logdebug(f"{dist_min=}")    
        return dist_min
    
    def calc_dist_multiplicator(self, q_l: List, q_r: List) -> float:
        """Calculates the distance multiplicator between two arms. This way, all of the distances can be used as one parameter in a cost function.

        Args:
            q_l (List): left joint_values
            q_r (List): right joint_values

        Returns:
            float: multiplicator
        """
        dist = self.calc_dist(q_l, q_r)
        m=1
        for d in dist:
            m *= d
        return m

    def compute_DH_matrix(self,theta,d,a,alpha):
        return [[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
                [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                [0, sin(alpha), cos(alpha), d],
                [0, 0, 0, 1]]


    def load_parameters(self):
        # self.ur_prefix_l = rospy.get_param('~ur_prefix_l', 'UR10_l/')
        # self.ur_prefix_r = rospy.get_param('~ur_prefix_r', 'UR10_r/')
        self.ur_prefix_l, self.ur_prefix_r = rospy.get_param('prefixs_ur', default=("UR10_l/", "UR10_r/"))
        self.tf_prefix = rospy.get_param('~tf_prefix', 'mur620')
        self.collision_objects_per_link = rospy.get_param('~collision_objects_per_link', 2)
        

if __name__ == '__main__':
    rospy.init_node('dual_arm_distance')
    cost = DualArmDistance()
    for i in range(10):
        min_dist = cost.calc_min_dist([i*np.pi/10,np.pi,0,0,0,0],[0,0,0,0,0,0])
        print(f"{min_dist=}")
    
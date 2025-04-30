#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import WrenchStamped, Pose, Wrench, Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import moveit_commander
import sys
from tf import transformations
import numpy as np
from controller_manager_msgs.srv import SwitchController

class Admittance_control():

    def __init__(self):
        rospy.init_node('admittance_control_node')
        self.get_params()
        joint_state_topic = ['joint_states:=' + self.joint_states_topic]
        moveit_commander.roscpp_initialize(joint_state_topic)
        self.actual_wrench = Wrench()
        self.target_wrench = Wrench()
        self.target_pose = Pose()
        self.actual_pose = Pose()
        self.feed_forward_velocity_cartesian = Twist()
        self.timestamp_old = rospy.get_rostime()
        self.q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        group_name = "UR_arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        rospy.Subscriber(self.wrench_topic, WrenchStamped, self.wrench_callback)
        rospy.Subscriber(self.target_pose_topic, Pose, self.target_pose_callback)
        rospy.Subscriber(self.actual_pose_topic, Pose, self.actual_pose_callback)
        rospy.Subscriber(self.target_wrench_topic, WrenchStamped, self.target_wrench_callback)
        rospy.Subscriber(self.cartesian_ff_velocity_topic, Twist, self.cartesian_ff_velocity_callback)
        rospy.Subscriber(self.joint_states_topic, JointState, self.joint_states_callback)
        rospy.sleep(1.0) # wait for the subscribers to be ready
        self.joint_group_vel_pub = rospy.Publisher(self.joint_group_vel_topic, Float64MultiArray, queue_size=1)
        

    def update(self):
        
        # change the controller to joint_group_vel_controller (the controller generelly should be switched somewhere else)
        stop = self.ur_prefix + 'arm_controller'
        start = 'joint_group_vel_controller_l/unsafe'
        self.switch_controllers(stop, start)

        # set current pose as target pose
        self.target_pose = self.actual_pose

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():

            # Calculate the desired force
            desired_compliance = self.desired_compliance(self.target_wrench, self.actual_wrench)
            # rospy.loginfo("Desired compliance: {}".format(desired_compliance))

            # Update the target pose
            target_pose = self.update_target_pose(self.target_pose, self.feed_forward_velocity_cartesian)
            # rospy.loginfo("Target pose: {}".format(target_pose))

            # Calculate the error between the current pose and the target pose
            pose_error = self.pose_error(self.actual_pose, target_pose, desired_compliance)
            rospy.loginfo("Pose error: {}".format(pose_error))

            # Calculate the target tcp velocity
            target_tcp_velocity = self.calculate_target_tcp_velocity(pose_error, self.feed_forward_velocity_cartesian)
            #rospy.loginfo("Target tcp velocity: {}".format(target_tcp_velocity))

            # Convert the target tcp velocity to joint velocity
            target_joint_velocity = self.tcp_velocity_to_joint_velocity(target_tcp_velocity)

            # Send the target joint velocity to the robot
            self.send_target_joint_velocity(target_joint_velocity)

            rate.sleep()

    def desired_compliance(self, target_wrench, current_wrench):
        desired_compliance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        desired_compliance[0] = (target_wrench.force.x - current_wrench.force.x) * self.admittance_gain_force_x
        desired_compliance[1] = (target_wrench.force.y - current_wrench.force.y) * self.admittance_gain_force_y
        desired_compliance[2] = (target_wrench.force.z - current_wrench.force.z) * self.admittance_gain_force_z
        desired_compliance[3] = (target_wrench.torque.x - current_wrench.torque.x) * self.admittance_gain_torque_x
        desired_compliance[4] = (target_wrench.torque.y - current_wrench.torque.y) * self.admittance_gain_torque_y
        desired_compliance[5] = (target_wrench.torque.z - current_wrench.torque.z) * self.admittance_gain_torque_z
        return desired_compliance
  
    def update_target_pose(self, target_pose, feed_forward_velocity_cartesian):
        now = rospy.get_rostime()
        dt = (now - self.timestamp_old).to_sec()
        target_pose.position.x = target_pose.position.x + feed_forward_velocity_cartesian.linear.x * dt
        target_pose.position.y = target_pose.position.y + feed_forward_velocity_cartesian.linear.y * dt
        target_pose.position.z = target_pose.position.z + feed_forward_velocity_cartesian.linear.z * dt
        # convert quaternion to euler angles to add the feed forward velocity
        [roll, pitch, yaw] = transformations.euler_from_quaternion([target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w])
        roll = roll + feed_forward_velocity_cartesian.angular.x * dt
        pitch = pitch + feed_forward_velocity_cartesian.angular.y * dt
        yaw = yaw + feed_forward_velocity_cartesian.angular.z * dt
        # convert euler angles to quaternion to update the target pose
        [target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w] = transformations.quaternion_from_euler(roll, pitch, yaw)
        self.timestamp_old = now
        return target_pose


    def pose_error(self, actual_pose, target_pose, desired_compliance):
        desired_compliance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pose_error = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pose_error[0]   = desired_compliance[0] + target_pose.position.x - actual_pose.position.x 
        pose_error[1]   = desired_compliance[1] + target_pose.position.y - actual_pose.position.y
        pose_error[2]   = desired_compliance[2] + target_pose.position.z - actual_pose.position.z
        # convert quaternion to euler angles
        [act_roll, act_pitch, act_yaw] = transformations.euler_from_quaternion([actual_pose.orientation.x, actual_pose.orientation.y, actual_pose.orientation.z, actual_pose.orientation.w])
        [tar_roll, tar_pitch, tar_yaw] = transformations.euler_from_quaternion([target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w])
        pose_error[3]   = desired_compliance[3] + tar_roll - act_roll
        pose_error[4]   = desired_compliance[4] + tar_pitch - act_pitch
        pose_error[5]   = desired_compliance[5] + tar_yaw - act_yaw
        return pose_error

    def calculate_target_tcp_velocity(self, pose_error, feed_forward_velocity_cartesian):
        target_tcp_velocity = Twist()
        target_tcp_velocity.linear.x = pose_error[0] * self.propotional_gain_pose_x + feed_forward_velocity_cartesian.linear.x
        target_tcp_velocity.linear.y = pose_error[1] * self.propotional_gain_pose_y + feed_forward_velocity_cartesian.linear.y
        target_tcp_velocity.linear.z = pose_error[2] * self.propotional_gain_pose_z + feed_forward_velocity_cartesian.linear.z
        target_tcp_velocity.angular.x = pose_error[3] * self.propotional_gain_pose_roll + feed_forward_velocity_cartesian.angular.x
        target_tcp_velocity.angular.y = pose_error[4] * self.propotional_gain_pose_pitch + feed_forward_velocity_cartesian.angular.y
        target_tcp_velocity.angular.z = pose_error[5] * self.propotional_gain_pose_yaw + feed_forward_velocity_cartesian.angular.z
        return target_tcp_velocity

    def tcp_velocity_to_joint_velocity(self, target_tcp_velocity_base):
        
        # get current joint positions
        #q = self.move_group.get_current_joint_values()
        # get jacobi matrix
        jacobian = self.move_group.get_jacobian_matrix(self.q)


        # convert target velocity from base to tcp
        target_tcp_velocity = self.base_to_tcp_velocity(target_tcp_velocity_base)
        target_tcp_velocity = target_tcp_velocity_base

        # calculate the inverse of the jacobian matrix
        jacobian_pinv = np.linalg.inv(jacobian)
        # calculate the joint velocity
        target_joint_velocity_unsorted = np.dot(jacobian_pinv, np.array([target_tcp_velocity.linear.x, target_tcp_velocity.linear.y, target_tcp_velocity.linear.z, target_tcp_velocity.angular.x, target_tcp_velocity.angular.y, target_tcp_velocity.angular.z]))
        # sort the joint velocity #TODO: Do this dynamically
        target_joint_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        target_joint_velocity[0] = target_joint_velocity_unsorted[2]
        target_joint_velocity[1] = target_joint_velocity_unsorted[1]
        target_joint_velocity[2] = target_joint_velocity_unsorted[0]
        target_joint_velocity[3] = target_joint_velocity_unsorted[3]
        target_joint_velocity[4] = target_joint_velocity_unsorted[4]
        target_joint_velocity[5] = target_joint_velocity_unsorted[5]
        return target_joint_velocity_unsorted

    def send_target_joint_velocity(self,target_joint_velocity):
        command = Float64MultiArray()
        command.data = target_joint_velocity
        self.joint_group_vel_pub.publish(command)

    def base_to_tcp_velocity(self, target_tcp_velocity_base):
        # get transformation matrix from base to tcp
        R = transformations.quaternion_matrix([self.actual_pose.orientation.x, self.actual_pose.orientation.y, self.actual_pose.orientation.z, self.actual_pose.orientation.w])
        T = np.identity(4)
        T[0:3,0:3] = R[0:3,0:3]
        T[0,3] = self.actual_pose.position.x
        T[1,3] = self.actual_pose.position.y
        T[2,3] = self.actual_pose.position.z
        T_inv = np.linalg.inv(T)
        # convert target velocity from base to tcp
        target_tcp_velocity = Twist()
        target_tcp_velocity.linear.x = T_inv[0,0] * target_tcp_velocity_base.linear.x + T_inv[0,1] * target_tcp_velocity_base.linear.y + T_inv[0,2] * target_tcp_velocity_base.linear.z
        target_tcp_velocity.linear.y = T_inv[1,0] * target_tcp_velocity_base.linear.x + T_inv[1,1] * target_tcp_velocity_base.linear.y + T_inv[1,2] * target_tcp_velocity_base.linear.z
        target_tcp_velocity.linear.z = T_inv[2,0] * target_tcp_velocity_base.linear.x + T_inv[2,1] * target_tcp_velocity_base.linear.y + T_inv[2,2] * target_tcp_velocity_base.linear.z
        target_tcp_velocity.angular.x = T_inv[0,0] * target_tcp_velocity_base.angular.x + T_inv[0,1] * target_tcp_velocity_base.angular.y + T_inv[0,2] * target_tcp_velocity_base.angular.z
        target_tcp_velocity.angular.y = T_inv[1,0] * target_tcp_velocity_base.angular.x + T_inv[1,1] * target_tcp_velocity_base.angular.y + T_inv[1,2] * target_tcp_velocity_base.angular.z
        target_tcp_velocity.angular.z = T_inv[2,0] * target_tcp_velocity_base.angular.x + T_inv[2,1] * target_tcp_velocity_base.angular.y + T_inv[2,2] * target_tcp_velocity_base.angular.z
        return target_tcp_velocity

    def cartesian_ff_velocity_callback(self, Twist):
        self.feed_forward_velocity_cartesian = Twist

    def wrench_callback(self, msg: WrenchStamped):
        self.actual_wrench = msg.wrench

    def target_pose_callback(self, msg):
        self.target_pose = msg

    def actual_pose_callback(self, msg):
        self.actual_pose = msg

    def target_wrench_callback(self, msg):
        self.target_wrench = msg.wrench

    def joint_states_callback(self,JointState):
        # TODO: sort the joint states only on the first run. Save the order in a list and use it afterwards
        for i in range(0,6):
            for idx in range(0,len(JointState.name)):
                if JointState.name[idx] == self.ur_prefix + self.joint_names[i]:
                    self.q[i] = JointState.position[idx]

    def get_params(self):
        self.wrench_topic = rospy.get_param("~actual_wrench_topic")
        self.target_pose_topic = rospy.get_param("~target_pose_topic")
        self.actual_pose_topic = rospy.get_param("~actual_pose_topic")
        self.target_wrench_topic = rospy.get_param("~target_wrench_topic")
        self.cartesian_ff_velocity_topic = rospy.get_param("~cartesian_ff_velocity_topic")
        self.joint_states_topic = rospy.get_param("~joint_states_topic")
        self.joint_group_vel_topic = rospy.get_param("~joint_group_vel_topic")
        self.propotional_gain_pose_x = rospy.get_param("~propotional_gain_pose_x")
        self.propotional_gain_pose_y = rospy.get_param("~propotional_gain_pose_y")
        self.propotional_gain_pose_z = rospy.get_param("~propotional_gain_pose_z")
        self.propotional_gain_pose_roll = rospy.get_param("~propotional_gain_pose_roll")
        self.propotional_gain_pose_pitch = rospy.get_param("~propotional_gain_pose_pitch")
        self.propotional_gain_pose_yaw = rospy.get_param("~propotional_gain_pose_yaw")
        self.admittance_gain_force_x = rospy.get_param("~admittance_gain_force_x")
        self.admittance_gain_force_y = rospy.get_param("~admittance_gain_force_y")
        self.admittance_gain_force_z = rospy.get_param("~admittance_gain_force_z")
        self.admittance_gain_torque_x = rospy.get_param("~admittance_gain_torque_x")
        self.admittance_gain_torque_y = rospy.get_param("~admittance_gain_torque_y")
        self.admittance_gain_torque_z = rospy.get_param("~admittance_gain_torque_z")
        self.joint_names = rospy.get_param("~joint_names")
        self.ur_prefix = rospy.get_param("~ur_prefix")

    def switch_controllers(self,stop, start):
        rospy.wait_for_service('/mur620/controller_manager/switch_controller')
        try:
            switch_controller = rospy.ServiceProxy('/mur620/controller_manager/switch_controller', SwitchController)
            switch_controller(start_controllers=[start], stop_controllers=[stop], strictness=2)
            return True
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False


if __name__ == '__main__':
    admittance_control = Admittance_control()
    admittance_control.update()
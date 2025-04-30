#!/usr/bin/env python3

# given a target position in the world frame and local target velocity, this controller calculates the velocity commands for the robot to reach the target position

import rospy
from geometry_msgs.msg import Pose, Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf import transformations, broadcaster
import math
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from copy import deepcopy
import numpy as np

class DecentralizedLeaderFollowerController:
    
    def __init__(self) -> None:
        self.config()
        self.setup_ddynamic_reconfigure()
        rospy.Subscriber(self.leader_pose_topic, PoseStamped, self.target_pose_callback)
        rospy.Subscriber(self.actual_pose_topic, PoseStamped, self.actual_pose_callback)
        rospy.Subscriber(self.leader_velocity_topic, Twist, self.target_velocity_callback)
        self.target_pose_broadcaster = broadcaster.TransformBroadcaster()
        self.cmd_vel_publisher = rospy.Publisher(self.follower_cmd_vel_topic, Twist, queue_size=10)
        self.cmd_vel_output = Twist()
        self.largest_error = [0.0,0.0,0.0] # used for logging - remove later
        self.target_pose_old = Pose()
        self.e_x_integrated = 0.0
        self.dphi_integrated = 0.0

    def config(self):
        self.leader_pose_topic = rospy.get_param("~leader_pose_topic", "/target_pose")
        self.actual_pose_topic = rospy.get_param("~actual_pose_topic", "/actual_pose")
        self.leader_velocity_topic = rospy.get_param("~leader_velocity_topic", "/target_velocity")
        self.follower_cmd_vel_topic = rospy.get_param("~follower_cmd_vel_topic", "/cmd_vel")
        self.tf_prefix = rospy.get_param("~tf_prefix", "follower")
        self.control_rate = rospy.get_param("~control_rate", 100)
        self.Ki_x = rospy.get_param("~Ki_x", 0.01)
        self.Kp_x = rospy.get_param("~Kp_x", 0.0)
        self.Kp_y = rospy.get_param("~Kp_y", 0.0)
        self.Kp_phi = rospy.get_param("~Kp_phi", 0.0)
        self.relative_position = rospy.get_param("~relative_position", [0.0,0.0,0.0])
        self.lin_vel_max = rospy.get_param("~lin_vel_max", 0.2)
        self.ang_vel_max = rospy.get_param("~ang_vel_max", 0.3)
        self.e_x_integrated_max = rospy.get_param("~e_x_integrated_max", 3.0)
        self.dphi_integrated_max = rospy.get_param("~dphi_integrated_max", 3.0)
        self.drive_backwards = rospy.get_param("~drive_backwards", False)
        rospy.loginfo("Kp_x: " + str(self.Kp_x))
        rospy.loginfo("Kp_y: " + str(self.Kp_y))
        rospy.loginfo("Kp_phi: " + str(self.Kp_phi))
        rospy.set_param("~test", self.Kp_x)

    def run(self):
        
        # wait for target pose, actual pose and target velocity
        rospy.wait_for_message(self.leader_pose_topic, PoseStamped)
        rospy.wait_for_message(self.actual_pose_topic, PoseStamped)
        rospy.wait_for_message(self.leader_velocity_topic, Twist)
        rospy.loginfo("Got target pose, actual pose and target velocity")
        
        # initialize old target pose
        self.target_pose_old = deepcopy(self.target_pose)

        self.rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            if self.target_pose is not None and self.actual_pose is not None and self.target_velocity is not None:
                u_v, u_w = self.compute_feed_forward_velocity()
            else:
                u_v = 0.0
                u_w = 0.0
                rospy.loginfo("Waiting for target pose, actual pose and target velocity")
            
            # limit the velocities
            if abs(u_v) > self.lin_vel_max:
                u_v = self.lin_vel_max * u_v/abs(u_v)
            if abs(u_w) > self.ang_vel_max:
                u_w = self.ang_vel_max * u_w/abs(u_w)
            
            self.cmd_vel_output.linear.x = u_v
            self.cmd_vel_output.angular.z = u_w
            self.cmd_vel_publisher.publish(self.cmd_vel_output)
            self.rate.sleep()
            
    def compute_feed_forward_velocity(self):
        
        # compute target pose in the world frame based on leader pose and relative position
        target_pose = deepcopy(self.target_pose)
        phi = transformations.euler_from_quaternion([self.target_pose.orientation.x,self.target_pose.orientation.y,self.target_pose.orientation.z,self.target_pose.orientation.w])[2]
        
        # position
        target_pose.position.x = self.target_pose.position.x + self.relative_position[0]*math.cos(phi) - self.relative_position[1]*math.sin(phi)
        target_pose.position.y = self.target_pose.position.y + self.relative_position[0]*math.sin(phi) + self.relative_position[1]*math.cos(phi)
        
        # compute follower velocity in world frame
        local_velocity_x = self.target_velocity.linear.x - self.relative_position[1]*self.target_velocity.angular.z
        local_velocity_y = self.relative_position[0]*self.target_velocity.angular.z

        global_velocity_x = local_velocity_x * math.cos(phi) - local_velocity_y * math.sin(phi)
        global_velocity_y = local_velocity_x * math.sin(phi) + local_velocity_y * math.cos(phi)

        phi = transformations.euler_from_quaternion([self.target_pose.orientation.x,self.target_pose.orientation.y,self.target_pose.orientation.z,self.target_pose.orientation.w])[2] + self.relative_position[2]
        
        if self.relative_position[0] == 0.0:
            target_pose.orientation = deepcopy(self.target_pose.orientation)
        elif self.relative_position[0] == 0.0:
            target_pose.orientation = deepcopy(self.target_pose.orientation)
        else:
            # compute target angle
            if  self.target_velocity.angular.z == 0.0 and self.target_velocity.linear.x == 0.0:
                # keep old target orientation if the robot is not moving to avoid jittering
                target_pose.orientation = self.target_pose_old.orientation
            elif self.target_velocity.angular.z == 0.0:
                target_pose.orientation = deepcopy(self.target_pose.orientation)
            else:
                target_angle = math.atan2(global_velocity_y,global_velocity_x) #* self.sign(self.target_velocity.angular.z) #+ math.pi / 2 
                    
                q = transformations.quaternion_from_euler(0.0, 0.0, target_angle)
                target_pose.orientation.x = q[0]
                target_pose.orientation.y = q[1]
                target_pose.orientation.z = q[2]
                target_pose.orientation.w = q[3]

        # compute the target velocity in the world frame based on leader velocity and relative position
        target_velocity = deepcopy(self.target_velocity)

        # compute linear feedforward velocity
        if self.relative_position[0] == 0.0:
            target_velocity.linear.x = self.target_velocity.linear.x + self.relative_position[1]*self.target_velocity.angular.z * self.psign(self.relative_position[1])
        elif self.relative_position[1] == 0.0:
            target_velocity.linear.x = self.target_velocity.linear.x + abs(self.relative_position[0]* self.target_velocity.angular.z) #* self.psign(self.relative_position[0])

        
        # compute angular feedforward velocity by comparing the new and old target pose
        phi_target = transformations.euler_from_quaternion([target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w])[2]
        phi_target_old = transformations.euler_from_quaternion([self.target_pose_old.orientation.x,self.target_pose_old.orientation.y,self.target_pose_old.orientation.z,self.target_pose_old.orientation.w])[2]
        pi_jump = self.detect_pi_jump(phi_target, phi_target_old)
        if pi_jump == True:
            # prevent the pi jump from being considered as a large change in orientation
            phi_target_old = phi_target

        # adjust the target angle for the formations angular velocity, this is done so only the changes relative to the leader orientation are considered
        phi_target_adjusted = phi_target - self.target_velocity.angular.z * self.rate.sleep_dur.to_sec()
        # smooth the orientation change as the robot should not change its orientation too fast
        self.dphi_integrated = self.smooth_derivative(phi_target_adjusted, phi_target_old, self.dphi_integrated, self.dphi_integrated_max, 0.01)
        target_velocity.angular.z = self.target_velocity.angular.z + self.dphi_integrated 

        u_v, u_w = self.cartesian_controller(self.actual_pose, target_pose, target_velocity)

        # update old target pose
        self.target_pose_old = deepcopy(target_pose)

        return u_v, u_w        
    
            
    def cartesian_controller(self,actual_pose = Pose(),target_pose = Pose(),target_velocity = Twist()):
            phi_act = transformations.euler_from_quaternion([actual_pose.orientation.x,actual_pose.orientation.y,actual_pose.orientation.z,actual_pose.orientation.w])[2]
            phi_target = transformations.euler_from_quaternion([target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w])[2]
            
            if self.drive_backwards == True:
                phi_act = phi_act + math.pi
                # wrap to [-pi,pi] interval
                if phi_act > math.pi:
                    phi_act = phi_act - 2*math.pi
                elif phi_act < -math.pi:
                    phi_act = phi_act + 2*math.pi

            e_x = (target_pose.position.x- actual_pose.position.x)
            e_y = (target_pose.position.y - actual_pose.position.y)
            e_phi = phi_target-phi_act

            # wrap to [-pi,pi] interval
            if e_phi > math.pi:
                e_phi = e_phi - 2*math.pi
            elif e_phi < -math.pi:
                e_phi = e_phi + 2*math.pi
            
            # broadcast actual pose
            self.target_pose_broadcaster.sendTransform((actual_pose.position.x, actual_pose.position.y, 0.0),
                                                (actual_pose.orientation.x, actual_pose.orientation.y, actual_pose.orientation.z, actual_pose.orientation.w),
                                                rospy.Time.now(),
                                                self.tf_prefix + "/actual_pose",
                                                "map")

            # broadcast target pose
            self.target_pose_broadcaster.sendTransform((target_pose.position.x, target_pose.position.y, 0.0),
                                                (target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w),
                                                rospy.Time.now(),
                                                self.tf_prefix + "/target_pose_control",
                                                "map")
                       
            e_local_x = e_x * math.cos(phi_target) + e_y * math.sin(phi_target)
            e_local_y = -e_x * math.sin(phi_target) + e_y * math.cos(phi_target)
            self.e_x_integrated += e_local_x
            if abs(self.e_x_integrated) > self.e_x_integrated_max:
                self.e_x_integrated = self.e_x_integrated_max * self.e_x_integrated/abs(self.e_x_integrated)

            # if the angular error is larger than 90 degrees, the robot should drive backwards
            if abs(e_phi) > math.pi/2 and self.relative_position[0] != 0.0:
                e_phi = e_phi - math.pi
                # wrap to [-pi,pi] interval
                if e_phi > math.pi:
                    e_phi = e_phi - 2*math.pi
                elif e_phi < -math.pi:
                    e_phi = e_phi + 2*math.pi
                backwards = True
            else:
                backwards = False

            u_v = target_velocity.linear.x * math.cos(e_phi) + self.Kp_x*e_local_x + self.Ki_x*self.e_x_integrated

            # if the robot should drive backwards, the linear velocity should be negative
            # if self.drive_backwards == True:
            #     u_v = -u_v

            u_w = target_velocity.angular.z +  self.Kp_y * e_local_y * self.psign(u_v) + self.Kp_phi * math.sin(e_phi) 
           
            if self.drive_backwards == True:
                # u_w = -u_w
                u_v = -u_v

            # if the robot should drive backwards, the linear velocity should be negative
            if backwards == True:
                u_v = -u_v
                u_w = -u_w
            
            
            # publish metadata
            # self.metadata_publisher.publish_controller_metadata(target_pose = target_pose, actual_pose = actual_pose, target_velocity = target_velocity, publish = True,
            # error = [e_local_x,e_local_y,phi_target[2]-phi_act[2]], robot_id = i) 

            return u_v, u_w
        
    def target_pose_callback(self, msg):
        self.target_pose = msg.pose    
    
    def actual_pose_callback(self, msg):
        self.actual_pose = msg.pose  
    
    def target_velocity_callback(self, msg):
        self.target_velocity = msg

    def psign(self, x):
        if x >= 0:
            return 1
        else:
            return -1
        
    def nsign(self, x):
        if x >= 0:
            return -1
        else:
            return 1
        
    def smooth_derivative(self, value, value_old, integral, max_integral, decay):
        # compute derivative and filter it with an exponential decay
        derivative = (value - value_old)
        integral = integral * (1-decay) + derivative 
        if abs(integral) > max_integral:
            integral = max_integral * integral/abs(integral)
        return integral

    def detect_pi_jump(self, value, value_old):
        # detect if the value jumps from -pi to pi or vice versa
        if value - value_old > math.pi and value > math.pi/2 and value_old < -math.pi/2:
            return True
        elif value - value_old < -math.pi and value < -math.pi/2 and value_old > math.pi/2:
            return True
        else:
            return False
        
    def setup_ddynamic_reconfigure(self):
        # Create a D(ynamic)DynamicReconfigure
        ddynrec = DDynamicReconfigure("decentralized_leader_follower_control")

        # Add variables (name, description, default value, min, max, edit_method)
        ddynrec.add_variable("Kp_x", "float/double variable", self.Kp_x, 0.0, self.Kp_x * 5)
        ddynrec.add_variable("Kp_y", "float/double variable", self.Kp_y, 0.0, self.Kp_y * 5)
        ddynrec.add_variable("Kp_phi", "float/double variable", self.Kp_phi, 0.0, self.Kp_phi * 5)
        ddynrec.add_variable("lin_vel_max", "float/double variable", self.lin_vel_max, 0.0, self.lin_vel_max * 5)
        ddynrec.add_variable("ang_vel_max", "float/double variable", self.ang_vel_max, 0.0, self.ang_vel_max * 5)
        ddynrec.add_variable("Ki_x", "float/double variable", self.Ki_x, 0.0, self.Ki_x * 5)
        #ddynrec.add_variable("servo_position", "integer variable", self.servo_position, 0, 1200)
        

        # Start the server
        ddynrec.start(self.dyn_rec_callback)
        
        
    def dyn_rec_callback(self,config, level):
        self.Kp_x = config["Kp_x"]
        self.Kp_y = config["Kp_y"]
        self.Kp_phi = config["Kp_phi"]
        self.lin_vel_max = config["lin_vel_max"]
        self.ang_vel_max = config["ang_vel_max"]
        self.Ki_x = config["Ki_x"]

        return config
        
if __name__ == '__main__':
    try:
        rospy.init_node('decentralized_leader_follower_controller', anonymous=True)
        controller = DecentralizedLeaderFollowerController().run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
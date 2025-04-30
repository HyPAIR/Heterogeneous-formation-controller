#!/usr/bin/env python3

import rospy
import yaml
import os
from nav_msgs.msg import Path 
from geometry_msgs.msg import Pose, PoseStamped, Twist
from tf import transformations, broadcaster
import math
from helper_nodes.metadata_publisher import Metadata_publisher
from copy import deepcopy


class Formation_controller_node():

    def __init__(self):
        self.path_array = rospy.get_param("~path_array", [])
        self.relative_positions_x = rospy.get_param("~relative_positions_x", [0, 2.0, -2.0])
        self.relative_positions_y = rospy.get_param("~relative_positions_y", [0, 0.0 , 0.0])
        self.robot_names = rospy.get_param("~robot_names", ["mir1", "mir2", "mir3"])
        self.mir_poses = [Pose() for i in range(len(self.robot_names))]
        self.target_pose_broadcaster = broadcaster.TransformBroadcaster()
        self.current_vel = 0.0
        self.current_omega = 0.0
        self.Kx = rospy.get_param("~Kx", 0.2)
        self.Ky = rospy.get_param("~Ky", 0.4)
        self.Kphi = rospy.get_param("~Kphi", 0.1)
        self.KP_vel = 1.0
        self.KP_omega = 1.0
        self.control_rate = rospy.get_param("~control_rate", 100.0)
        self.control_rate = 100
        self.velocity_limit_lin = rospy.get_param("~velocity_limit_lin", 0.5)
        self.velocity_limit_lin = 1.0
        self.velocity_limit_ang = rospy.get_param("~velocity_limit_ang", 1.5)
        self.velocity_limit_ang = 1.5
        self.acceleration_limit_lin = rospy.get_param("~acceleration_limit_lin", 2.0)
        self.acceleration_limit_lin = 2.0
        self.acceleration_limit_ang = rospy.get_param("~acceleration_limit_ang", 2.5)
        self.acceleration_limit_ang = 2.5
        self.robot_path_publishers = []
        self.robot_twist_publishers = []
        self.metadata_publisher = Metadata_publisher()
        
        for i in range(len(self.robot_names)):
            self.robot_path_publishers.append(rospy.Publisher(self.robot_names[i] + '/robot_path', Path, queue_size=1))
            self.robot_twist_publishers.append(rospy.Publisher(self.robot_names[i] + '/mobile_base_controller/cmd_vel', Twist, queue_size=1))

        for i in range(len(self.robot_names)):
            rospy.Subscriber(self.robot_names[i] + '/mir_pose_simple', Pose, self.mir_pose_callback, i)   
        
        rospy.sleep(1.0)


    def run(self):
        self.derive_robot_paths()
        self.compute_path_lengths()
        # init variables
        path_index = 1
        path_index_old = 0
        distances = [0.0 for i in range(len(self.robot_names))]
        current_distances = [0.0 for i in range(len(self.robot_names))]
        distances_old = [10e10 for i in range(len(self.robot_names))] # set this very high to ensure that the first point is always reached
        target_vels = [0.0 for i in range(len(self.robot_names))]
        target_points = [[0.0, 0.0] for i in range(len(self.robot_names))]
        target_angles = [0.0 for i in range(len(self.robot_names))]
        target_omegas = [0.0 for i in range(len(self.robot_names))]
        target_poses = [Pose() for i in range(len(self.robot_names))]
        current_vels = [0.0 for i in range(len(self.robot_names))]
        current_omegas = [0.0 for i in range(len(self.robot_names))]
        current_thetas = [0.0 for i in range(len(self.robot_names))]



        # init target poses depending on relative positions
        for i in range(len(self.robot_names)):
            target_poses[i].position.x = self.robot_paths_x[i][0]
            target_poses[i].position.y = self.robot_paths_y[i][0]
            target_poses[i].position.z = 0.0
            q = transformations.quaternion_from_euler(0, 0, self.path_array[0][2])
            target_poses[i].orientation.x = q[0]
            target_poses[i].orientation.y = q[1]
            target_poses[i].orientation.z = q[2]    
            target_poses[i].orientation.w = q[3]
            current_thetas[i] = self.path_array[0][2]

        rate = rospy.Rate(self.control_rate)
        timestamp_old = rospy.Time.now()
        # main loop
        while path_index < len(self.path_array)-2 and not rospy.is_shutdown() :
            # compute distance to next point
            for i in range(len(self.robot_names)):
                #overall_distances[i] += math.sqrt((self.robot_paths_x[i][path_index] - self.robot_paths_x[i][path_index - 1])**2 + (self.robot_paths_y[i][path_index] - self.robot_paths_y[i][path_index - 1])**2)
                distances[i] = self.path_lengths[i][path_index] - current_distances[i]
                #distances[i] = math.sqrt((self.robot_paths_x[i][path_index] - target_poses[i].position.x)**2 + (self.robot_paths_y[i][path_index] - target_poses[i].position.y)**2)

            # compute target velocity
            for i in range(len(self.robot_names)):
                target_vels[i] = self.KP_vel * distances[i] * self.control_rate

            # limit target velocity
            vel_scaling_factor = 1.0
            for i in range(len(self.robot_names)):
                if abs(target_vels[i]) > self.velocity_limit_lin:
                    if (self.velocity_limit_lin / abs(target_vels[i]) ) < vel_scaling_factor:
                        vel_scaling_factor =  self.velocity_limit_lin / abs(target_vels[i])

            # apply velocity scaling factor
            for i in range(len(self.robot_names)):
                target_vels[i] = target_vels[i] * vel_scaling_factor

            # limit target acceleration
            vel_scaling_factor = 1.0
            for i in range(len(self.robot_names)):
                acc_target = abs(target_vels[i] - current_vels[i]) 
                if acc_target > self.acceleration_limit_lin:
                    if (self.acceleration_limit_lin/acc_target) < vel_scaling_factor:
                        vel_scaling_factor = self.acceleration_limit_lin / acc_target

            # apply velocity scaling factor
            for i in range(len(self.robot_names)):
                target_vels[i] = target_vels[i] * vel_scaling_factor
            vel_scaling_factor = 1.0

            # check if next point is reached
            for i in range(len(self.robot_names)):
                if distances[i] <= abs(target_vels[i]) * rate.sleep_dur.to_sec(): 
                    path_index += 1
                    distances_old = deepcopy(distances)
                    break
                if distances[i] > distances_old[i] and path_index != path_index_old: # if distance is increasing, we have passed the target point and need to move on to the next one
                    path_index += 1
                    break
            distances_old = deepcopy(distances)
            path_index_old = path_index

            # compute next target point
            for i in range(len(self.robot_names)):
                # target_points[i][0] = self.robot_paths_x[i][path_index+2]*0.5 + self.robot_paths_x[i][path_index+1]*0.5
                # target_points[i][1] = self.robot_paths_y[i][path_index+2]*0.5 + self.robot_paths_y[i][path_index+1]*0.5
                target_points[i][0] = self.robot_paths_x[i][path_index]
                target_points[i][1] = self.robot_paths_y[i][path_index]

            # broadcast target point
            for i in range(len(self.robot_names)):
                self.target_pose_broadcaster.sendTransform((target_points[i][0], target_points[i][1], 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), self.robot_names[i] + '/target_point', 'map')

            # compute angle to target point
            for i in range(len(self.robot_names)):
                target_angles[i] = math.atan2(target_points[i][1] - target_poses[i].position.y, target_points[i][0] - target_poses[i].position.x)
                #target_angles[i] = math.atan2(target_points[i][1] - self.robot_paths_y[i][path_index-1], target_points[i][0] - self.robot_paths_x[i][path_index-1])

            # prevent the angle from jumping from -pi to pi
            for i in range(len(self.robot_names)):
                if target_angles[i] > math.pi:
                    target_angles[i] -= 2*math.pi
                elif target_angles[i] < -math.pi:
                    target_angles[i] += 2*math.pi

            # compute angle error
            for i in range(len(self.robot_names)):
                angle_error = target_angles[i] - current_thetas[i]
                if angle_error > math.pi:
                    angle_error -= 2*math.pi
                elif angle_error < -math.pi:
                    angle_error += 2*math.pi
                target_omegas[i] = self.KP_omega * angle_error

            # limit angular velocity
            for i in range(len(self.robot_names)):
                vel_target = abs(target_omegas[i]) #/ rate.sleep_dur.to_sec()
                if vel_target  > self.velocity_limit_ang:
                    if (self.velocity_limit_ang / vel_target) < vel_scaling_factor:
                        vel_scaling_factor = self.velocity_limit_ang / vel_target

            # apply velocity scaling factor
            for i in range(len(self.robot_names)):
                target_vels[i] *= vel_scaling_factor
                target_omegas[i] *= vel_scaling_factor
            vel_scaling_factor = 1.0

            # limit angular acceleration
            for i in range(len(self.robot_names)):
                acc_target = abs(target_omegas[i] - self.current_omega) 
                if acc_target > self.acceleration_limit_ang:
                    if (self.acceleration_limit_ang / acc_target) < vel_scaling_factor:
                        vel_scaling_factor = self.acceleration_limit_ang / acc_target

            # apply velocity scaling factor
            for i in range(len(self.robot_names)):
                target_vels[i] *= vel_scaling_factor
                target_omegas[i] *= vel_scaling_factor

            dt = rospy.Time.now() - timestamp_old
            # compute target pose for each robot
            for i in range(len(self.robot_names)):
                target_poses[i].position.x += target_vels[i] * math.cos(current_thetas[i]) * dt.to_sec()
                target_poses[i].position.y += target_vels[i] * math.sin(current_thetas[i]) * dt.to_sec()
                #q = transformations.quaternion_from_euler(0.0, 0.0, target_angles[i])
                q = transformations.quaternion_from_euler(0.0, 0.0, current_thetas[i] + target_omegas[i] * dt.to_sec())
                target_poses[i].orientation.x = q[0]
                target_poses[i].orientation.y = q[1]
                target_poses[i].orientation.z = q[2]
                target_poses[i].orientation.w = q[3]

            # broadcast target poses
            for i in range(len(self.robot_names)):
                self.target_pose_broadcaster.sendTransform((target_poses[i].position.x, target_poses[i].position.y, 0.0),
                                            (target_poses[i].orientation.x, target_poses[i].orientation.y, target_poses[i].orientation.z, target_poses[i].orientation.w),
                                            rospy.Time.now(),
                                            "target_pose_" + str(i),
                                            "map")

            dt = rospy.Time.now() - timestamp_old
            # update current velocities
            for i in range(len(self.robot_names)):
                current_vels[i] = target_vels[i]
                current_omegas[i] = target_omegas[i]
                current_thetas[i] += target_omegas[i] * dt.to_sec()
                current_distances[i] += target_vels[i] * dt.to_sec()
            timestamp_old = rospy.Time.now()

            # compute control law and publish target velocities
            for i in range(len(self.robot_names)):
                target_velocity = Twist()
                target_velocity.linear.x = target_vels[i]
                target_velocity.angular.z = target_omegas[i]
                u_v, v_w = self.cartesian_controller(self.mir_poses[i],target_poses[i],target_velocity,i)

                # publish target velocities
                target_velocity.linear.x = u_v
                target_velocity.angular.z = v_w
                self.robot_twist_publishers[i].publish(target_velocity)

            rate.sleep()

            

            
            

    def cartesian_controller(self,actual_pose = Pose(),target_pose = Pose(),target_velocity = Twist(),i = 0):
        phi_act = transformations.euler_from_quaternion([actual_pose.orientation.x,actual_pose.orientation.y,actual_pose.orientation.z,actual_pose.orientation.w])
        phi_target = transformations.euler_from_quaternion([target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w])
        R = transformations.quaternion_matrix([actual_pose.orientation.x,actual_pose.orientation.y,actual_pose.orientation.z,actual_pose.orientation.w])

        e_x = (target_pose.position.x- actual_pose.position.x)
        e_y = (target_pose.position.y - actual_pose.position.y)
        e_local_x = R[0,0]*e_x + R[1,0]*e_y
        e_local_y = R[0,1]*e_x + R[1,1]*e_y

        # broadcast actual pose
        self.target_pose_broadcaster.sendTransform((actual_pose.position.x, actual_pose.position.y, 0.0),
                                            (actual_pose.orientation.x, actual_pose.orientation.y, actual_pose.orientation.z, actual_pose.orientation.w),
                                            rospy.Time.now(),
                                            "actual_pose_" + str(i),
                                            "map")

        # broadcast target pose
        self.target_pose_broadcaster.sendTransform((target_pose.position.x, target_pose.position.y, 0.0),
                                            (target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w),
                                            rospy.Time.now(),
                                            "target_pose_controll" + str(i),
                                            "map")

        u_w = target_velocity.angular.z + target_velocity.linear.x * ( self.Ky * e_local_y + self.Kphi * math.sin(phi_target[2]-phi_act[2]))
        u_v = target_velocity.linear.x * math.cos(phi_target[2]-phi_act[2]) + self.Kx*e_local_x

        # publish metadata
        self.metadata_publisher.publish_controller_metadata(target_pose = target_pose, actual_pose = actual_pose, target_velocity = target_velocity, publish = True,
        error = [e_local_x,e_local_y,phi_target[2]-phi_act[2]], robot_id = i) 

        return u_v, u_w


    def compute_path_lengths(self):
        # compute path lengths
        self.path_lengths = [[0] for _ in range(len(self.robot_names))]
        for idx in range(0,len(self.robot_names)):
            for i in range(1,len(self.path_array)):
                self.path_lengths[idx].append(self.path_lengths[idx][i-1] + math.sqrt((self.robot_paths_x[idx][i]-self.robot_paths_x[idx][i-1])**2 + (self.robot_paths_y[idx][i]-self.robot_paths_y[idx][i-1])**2))


    def derive_robot_paths(self):
        # derive robot paths from path_array
        self.robot_paths_x = [ [] for i in range(len(self.robot_names))]
        self.robot_paths_y = [ [] for i in range(len(self.robot_names))]
        self.robot_paths_theta = [ [] for i in range(len(self.robot_names))]
        for idx in range(0,len(self.robot_names)):
            for i in range(len(self.path_array)):
                self.robot_paths_x[idx].append(self.path_array[i][0] + self.relative_positions_x[idx] * math.cos(self.path_array[i][2]) - self.relative_positions_y[idx] * math.sin(self.path_array[i][2]))
                self.robot_paths_y[idx].append(self.path_array[i][1] + self.relative_positions_x[idx] * math.sin(self.path_array[i][2]) + self.relative_positions_y[idx] * math.cos(self.path_array[i][2]))
                self.robot_paths_theta[idx].append(self.path_array[i][2])

        self.publish_robot_paths()

    def publish_robot_paths(self):
        # publish robot paths
        robot_path = Path()
        robot_path.header.frame_id = "map"
        data0 = []
        data1 = []
        data2 = []
        robot_path.header.stamp = rospy.Time.now()
        for i in range(len(self.robot_names)):
            robot_path = Path()
            robot_path.header.frame_id = "map"
            robot_path.header.stamp = rospy.Time.now()
            for j in range(len(self.robot_paths_x[i])):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = self.robot_paths_x[i][j]
                pose.pose.position.y = self.robot_paths_y[i][j]
                pose.pose.position.z = 0.0
                q = transformations.quaternion_from_euler(0.0, 0.0, self.robot_paths_theta[i][j])
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                robot_path.poses.append(pose)
            #     if (i == 0):
            #         data0.append([{'x': self.robot_paths_x[i][j], 'y': self.robot_paths_y[i][j], 'theta': self.robot_paths_theta[i][j]}])
            #     if (i == 1):
            #         data1.append([{'x': self.robot_paths_x[i][j], 'y': self.robot_paths_y[i][j], 'theta': self.robot_paths_theta[i][j]}])
            #     if (i == 2):
            #         data2.append([{'x': self.robot_paths_x[i][j], 'y': self.robot_paths_y[i][j], 'theta': self.robot_paths_theta[i][j]}])
            # if (i == 0):
            #     file_name = '/home/weijian/mir_robot_test/src/match_mobile_robotics/data11.yaml'
            #     with open(file_name, 'w') as file:
            #         yaml.dump(data0, file, default_flow_style=False)
            # if (i == 1):
            #     file_name = '/home/weijian/mir_robot_test/src/match_mobile_robotics/data11_1.yaml'
            #     with open(file_name, 'w') as file:
            #         yaml.dump(data1, file, default_flow_style=False)
            # if (i == 2):
            #     file_name = '/home/weijian/mir_robot_test/src/match_mobile_robotics/data11_2.yaml'
            #     with open(file_name, 'w') as file:
            #         yaml.dump(data2, file, default_flow_style=False)
            self.robot_path_publishers[i].publish(robot_path)
            rospy.sleep(0.5)

    def mir_pose_callback(self, msg, i):
        self.mir_poses[i] = msg
        # compute current theta
        


if __name__ == "__main__":
    try:
        rospy.init_node('formation_controller')
        rospy.loginfo('formation_controller node started')
        exe = Formation_controller_node()
        exe.run()
    except rospy.ROSInterruptException:
        pass
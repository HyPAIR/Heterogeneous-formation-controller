// Created by weijian on 17/11/23.

// include ROS Libraries
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <iostream>
#include "heterogeneous_formation_controller/vehicle_model.h"
#include "heterogeneous_formation_controller/optimizer_interface.h"
#include "yaml-cpp/yaml.h"
#include <vector>
#include "heterogeneous_formation_controller/math/pose.h"
#include "traj_tracking/fg_test.h"
#include "traj_tracking/matplotlibcpp.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "heterogeneous_formation_controller/yaml_all.h"

namespace plt = matplotlibcpp;

using namespace heterogeneous_formation_controller;
using namespace math;

// function to get robot position and orientation from odom topic
void plot_circle (const double& x, const double& y, int index, int pose_index);
void plot_car (const double& x, const double& y, const double& theta, int index, int pose_index);
void robot_odom1(const nav_msgs::Odometry msg);
std::vector<Pose> generate_path(std::string filename);
// Trajectory generate_traj(std::string filename);
Trajectory generate_ref_traj(Trajectory traj_lead, double offset);
Trajectory generate_ref_traj_diff(Trajectory traj_lead, double offset, double angle);
// function to calculate the euclidean distance between robot and desired traj 
double getDistance(double x1, double y1, double x2, double y2);

// variable declaration
// ackermann_msgs::AckermannDriveStamped robot1_cmd, robot2_cmd;
geometry_msgs::Twist robot3_cmd;
ros::Publisher robot3_cmd_pub; // node publishers
tf::Point Odom_pos;    //odometry position (x,y)
geometry_msgs::Pose actual_pose;
double Odom_yaw;    //odometry orientation (yaw)
geometry_msgs::Pose2D qd,robot_pose3, err;
geometry_msgs::Twist vel_msg;
double robot3_vel;
double robot3_omg;
std::vector<double> UpdateControlInputTwoWheel(int timestep, const std::vector<Pose>& path, const geometry_msgs::Pose2D& robot_pose, 
						double ev, double ew, ros::Publisher& robot_vel_pub,geometry_msgs::Twist& vel_msg);
void UpdateControlInputCar(int timestep, const Trajectory& traj, const geometry_msgs::Pose2D& robot_pose, 
						ros::Publisher& robot_vel_pub, geometry_msgs::Twist& vel_msg);
void zero_input(ros::Publisher& robot_vel_pub, geometry_msgs::Twist& vel_msg);

// VehicleModel vehicle;
// double vel_max = vehicle.max_vel_diff;
// double omg_max = vehicle.omg_max_diff;
// double acc_max = vehicle.max_acc_diff;
// double angacc_max = vehicle.omg_acc_diff;

// std::vector<std::vector<std::vector<double>>> generate_traj_fg(std::string filename, int numR) {
//     // 读取YAML文件
//     YAML::Node traj_ = YAML::LoadFile(filename);

//     // 检查文件是否成功加载
//     if (!traj_.IsDefined()) {
//         std::cerr << "Failed to load yaml file.\n";
//     }
//     int j = 0, k = 0;
//     std::vector<std::vector<std::vector<double>>> plan(numR, std::vector<std::vector<double>>(traj_.size() / numR , std::vector<double>(5)));
//     // 读取每个轨迹点
//     for (const auto& kv : traj_) {
//         const std::string& poseName = kv.first.as<std::string>();
//         const YAML::Node& poseNode = kv.second;
//         plan[j][k][0] = poseNode["x"].as<double>();
//         plan[j][k][1] = poseNode["y"].as<double>();
//         plan[j][k][2] = poseNode["theta"].as<double>();
//         plan[j][k][3] = poseNode["v"].as<double>();
//         plan[j][k][4] = poseNode["omega"].as<double>();
//         k++;
//         if (k > traj_.size() / numR - 1) {
//             j++;
//             k = 0;
//         }
//     }
//     return plan;
// }

std::vector<std::vector<Vec2d>> generate_obs(std::string filename) {
	std::vector<std::vector<Vec2d>> obstacle;
	Vec2d vert;
	// 读取YAML文件
	YAML::Node obs = YAML::LoadFile(filename);

	// 检查文件是否成功加载
	if (!obs.IsDefined()) {
		std::cerr << "Failed to load yaml file.\n";
	}

	// 读取每个轨迹点
	for (const auto& kv : obs) {
		std::vector<Vec2d> obs_;
		const std::string& poseName = kv.first.as<std::string>();
		const YAML::Node& poseNode = kv.second;
		double x = poseNode["vx1"].as<double>();
		double y = poseNode["vy1"].as<double>();
		vert.set_x(x);
		vert.set_y(y);
		obs_.push_back(vert);
		x = poseNode["vx2"].as<double>();
		y = poseNode["vy2"].as<double>();
		vert.set_x(x);
		vert.set_y(y);
		obs_.push_back(vert);			
		x = poseNode["vx3"].as<double>();
		y = poseNode["vy3"].as<double>();
		vert.set_x(x);
		vert.set_y(y);
		obs_.push_back(vert);			
		x = poseNode["vx4"].as<double>();
		y = poseNode["vy4"].as<double>();
		vert.set_x(x);
		vert.set_y(y);
		obs_.push_back(vert);
		obstacle.push_back(obs_);
	}
	return obstacle;
}

void zero_input(ros::Publisher& robot_vel_pub, geometry_msgs::Twist& vel_msg) {
	vel_msg.linear.x = 0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =0;
	robot_vel_pub.publish(vel_msg);
}

void robot_actual_pose3(const geometry_msgs::Pose msg)
{
	actual_pose.position.x = msg.position.x;
	actual_pose.position.y = msg.position.y;
	actual_pose.position.z = msg.position.z;
	actual_pose.orientation.x = msg.orientation.x;
	actual_pose.orientation.y = msg.orientation.y;
	actual_pose.orientation.z = msg.orientation.z;
	actual_pose.orientation.w = msg.orientation.w;
    tf::pointMsgToTF(msg.position, Odom_pos);
    Odom_yaw = tf::getYaw(msg.orientation);
	robot_pose3.x = msg.position.x;
	robot_pose3.y = msg.position.y;
    robot_pose3.theta = Odom_yaw;   
}

void plot_traj(std::vector<std::vector<double>> traj, std::vector<double> x_plot, std::vector<double>  y_plot, std::vector<double> theta_plot, std::vector<std::vector<Vec2d>> obstacle, const std::string& title) {
	std::vector<double> x_orig_plot, y_orig_plot, theta_orig_plot;
	std::map<std::string, std::string> keywords;
	keywords["color"] = "black";
	for (int i = 0; i < traj.size(); i++) {
		x_orig_plot.push_back(traj[i][0]);
		y_orig_plot.push_back(traj[i][1]);
		theta_orig_plot.push_back(traj[i][2]);
    }
	for (int i = 0; i < x_orig_plot.size(); i++) {
		plot_car(x_orig_plot[i], y_orig_plot[i], theta_orig_plot[i], 1, i);
	}
	for (int i = 0; i < x_plot.size(); i++) {
		plot_car(x_plot[i], y_plot[i], theta_plot[i], 2, i);
	}
	for (const auto& obs : obstacle) {
		std::vector<double> obs_x, obs_y;
		for (const auto& vert : obs) {
			obs_x.push_back(vert.x() / 10);
			obs_y.push_back(vert.y() / 10);
		}
		plt::fill(obs_x, obs_y, keywords);
	}
	plt::title(title); 
}

std::vector<double> cartesian_controller(const geometry_msgs::Pose& actual_pose, const double target_pose_x, const double target_pose_y, const double target_pose_theta,
										 const double target_vel, const double target_angvel) {
	std::vector<double> control_cmd;
	control_cmd.resize(2);
	double theta_act, e_x, e_y, e_local_x, e_local_y, u_w, u_v;
	theta_act = tf::getYaw(actual_pose.orientation);
	e_x = target_pose_x - actual_pose.position.x;
	e_y = target_pose_y - actual_pose.position.y;
	e_local_x = cos(theta_act) * e_x + sin(theta_act) * e_y;
	e_local_y = -sin(theta_act) * e_x + cos(theta_act) * e_y;
	u_w = target_angvel + target_vel * (Ky * e_local_y + Ktheta * sin(target_pose_theta - theta_act));
	u_v = target_vel * cos(target_pose_theta - theta_act) + Kx * e_local_x;
	control_cmd[0] = u_v;
	control_cmd[1] = u_w;
	return control_cmd;
}

std::vector<double> compute_tarj_length(const std::vector<std::vector<double>> path) {
	std::vector<double> path_lengths = {0};
	for (int i = 1; i < path.size(); i++) {
		path_lengths.push_back(path_lengths[i-1] + hypot(path[i][0] - path[i-1][0], path[i][1] - path[i-1][1]));
	}
	return path_lengths;
}

int main(int argc, char **argv)
{
	std::vector<double> x1_orig_plot, y1_orig_plot, theta1_orig_plot, x2_orig_plot, y2_orig_plot, theta2_orig_plot, x3_orig_plot, y3_orig_plot, theta3_orig_plot, x4_orig_plot, y4_orig_plot, theta4_orig_plot, x5_orig_plot, y5_orig_plot, theta5_orig_plot,
	x6_orig_plot, y6_orig_plot, theta6_orig_plot, x7_orig_plot, y7_orig_plot, theta7_orig_plot, x8_orig_plot, y8_orig_plot, theta8_orig_plot, x9_orig_plot, y9_orig_plot, theta9_orig_plot, x10_orig_plot, y10_orig_plot, theta10_orig_plot;
	std::vector<double> x1_plot, y1_plot, theta1_plot, x2_plot, y2_plot, theta2_plot, x3_plot, y3_plot, theta3_plot, x4_plot, y4_plot, theta4_plot, x5_plot, y5_plot, theta5_plot,
	x6_plot, y6_plot, theta6_plot, x7_plot, y7_plot, theta7_plot, x8_plot, y8_plot, theta8_plot, x9_plot, y9_plot, theta9_plot, x10_plot, y10_plot, theta10_plot;
    double distances = 0.0, current_distances = 0.0, distances_old = 0.0, current_vels = 0.0, current_omegas = 0.0, current_thetas = 0.0;
	double target_pose_x, target_pose_y, target_pose_theta, target_vels, target_angvels;
	ros::Time timestamp_old;
	ros::Duration dt;
	int robot_index = 2;
	auto plan =  generate_traj_fg("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/ras_demo/formation_generation/fg_gazebo.yaml", 10);
	// int robot_index = 0;
	// auto traj_1 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/ras_demo/result/diff_drive2.yaml");
	// std::vector<std::vector<std::vector<double>>> plan(1, std::vector<std::vector<double>>(traj_1.size() , std::vector<double>(5)));
	// for (int i = 0; i < traj_1.size(); i++) {
	// 	plan[0][i][0] = traj_1[i].x;
	// 	plan[0][i][1] = traj_1[i].y;
	// 	plan[0][i][2] = traj_1[i].theta;
	// 	plan[0][i][3] = traj_1[i].v;
	// 	plan[0][i][4] = traj_1[i].omega;
	// }	
	target_pose_x = plan[robot_index][0][0];
	target_pose_y = plan[robot_index][0][1];
	target_pose_theta = plan[robot_index][0][2];
	current_thetas = plan[robot_index][0][2];
	// std::vector<double> x, y;
    // int j = 3;
	// for (int i = 0; i < plan[j].size(); i++) {
	// 	x.push_back(i * 0.2);
	// 	y.push_back(plan[j][i][4]);
	// }
	// plt::plot(x, y);
	// plt::show();
	auto path_lengths = compute_tarj_length(plan[robot_index]);
	auto obstacle = generate_obs("/home/weijian/traj_tracking/src/obstacle.yaml");
	// initialization of ROS node
    ros::init(argc, argv, "robot_control_3");
    ros::NodeHandle n;
    ros::Subscriber sub_odometry_robot1 = n.subscribe("/mir3/mir_pose_simple", 1000 , robot_actual_pose3);
	ros::Rate loop_rate(100);
	robot3_cmd_pub = n.advertise<geometry_msgs::Twist>("/mir3/mobile_base_controller/cmd_vel", 1000);
	int path_index = 1;
	int path_index_old = 0;
	timestamp_old = ros::Time::now(); 
	// main loop
	while(path_index < plan[robot_index].size() - 2 && ros::ok()){
		// compute distance to next point
		distances = path_lengths[path_index] - current_distances;
		// compute target velocity
		target_vels = plan[robot_index][path_index][3];
		// target_vels = KP_vel * distances * control_rate;
		// check if next point is reached
		if (distances <= fabs(target_vels) * loop_rate.expectedCycleTime().toSec()) {
			path_index += 1;
			distances_old  = distances;
		}
		if (distances > distances_old && path_index != path_index_old) {
			path_index += 1;
		}
		if (hypot(robot_pose3.x - plan[robot_index].back()[0], robot_pose3.y - plan[robot_index].back()[1]) < 0.1 && fabs(robot_pose3.theta - plan[robot_index].back()[2]) < 0.1) {
			break;
		}
		distances_old = distances;
		path_index_old = path_index;
		// compute next target point
		target_pose_x = plan[robot_index][path_index][0];
		target_pose_y = plan[robot_index][path_index][1];
		target_pose_theta = plan[robot_index][path_index][2];
		if (target_pose_theta > M_PI) {
			target_pose_theta -= 2 * M_PI;
		}
		else if (target_pose_theta < -M_PI) {
			target_pose_theta += 2 * M_PI;
		}
		target_angvels = plan[robot_index][path_index][4];
		dt = ros::Time::now() - timestamp_old;
		current_vels = target_vels;
		current_omegas = target_angvels;
		current_thetas += target_angvels * dt.toSec();
		current_distances += fabs(target_vels) * dt.toSec();
		timestamp_old = ros::Time::now();
		if (actual_pose.position.x == 0 && actual_pose.position.y == 0) {
			robot3_cmd.linear.x = 0.0;
			robot3_cmd.angular.z = 0.0;			
		}
		else {
			robot3_cmd.linear.x = target_vels;
			robot3_cmd.angular.z = target_angvels;
			auto cmd = cartesian_controller(actual_pose, target_pose_x, target_pose_y, target_pose_theta, target_vels, target_angvels);
			robot3_cmd.linear.x = cmd[0];
			robot3_cmd.angular.z = cmd[1];
		}
		
		if (robot_pose3.x != 0 && robot_pose3.y != 0 && robot_pose3.theta != 0) {
			x1_plot.push_back(robot_pose3.x);
			y1_plot.push_back(robot_pose3.y);
			theta1_plot.push_back(robot_pose3.theta);
		}
		robot3_cmd_pub.publish(robot3_cmd);
		ros::spinOnce();
		loop_rate.sleep();
	}
	// plt::clf();
    // plot_traj(plan[robot_index], x1_plot, y1_plot, theta1_plot, obstacle, "Diff3 trajectory");
	// plt::show();
	std::cout << "robot 3 done!" << std::endl;
    return 0;
}
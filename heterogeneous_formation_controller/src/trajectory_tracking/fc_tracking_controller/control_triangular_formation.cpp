/**
 * file control_triangular_formation.cpp
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief heterogeneous formation controller (traingular)
 * data 2023-11-22
 * 
 * @copyright Copyroght(c) 2023
*/
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <iostream>
#include "heterogeneous_formation_controller/vehicle_model.h"
#include "yaml-cpp/yaml.h"
#include <vector>
#include "heterogeneous_formation_controller/math/pose.h"
#include "heterogeneous_formation_controller/optimizer_interface.h"
#include "traj_tracking/matplotlibcpp.h"
#include "traj_tracking/fg_test.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "gazebo_msgs/ModelStates.h"

namespace plt = matplotlibcpp;
using namespace heterogeneous_formation_controller;
using namespace math;

// function to get robot position and orientation from odom topic
void plot_circle (const double& x, const double& y, int index, int pose_index);
void plot_car (const double& x, const double& y, const double& theta, int index, int pose_index);
void robot_odom1(const nav_msgs::Odometry msg);
std::vector<Pose> generate_path(std::string filename);
Trajectory_temp generate_traj(std::string filename);
Trajectory_temp generate_ref_traj(Trajectory_temp traj_lead, double offset);
Trajectory_temp generate_ref_traj_diff(Trajectory_temp traj_lead, double offset, double angle);
// function to calculate the euclidean distance between robot and desired traj 
double getDistance(double x1, double y1, double x2, double y2);

// variable declaration
// ackermann_msgs::AckermannDriveStamped robot1_cmd, robot2_cmd;
std::vector<geometry_msgs::Twist> robots_cmd;
ros::Publisher robot_vel_pub, robot1_cmd_pub, robot2_cmd_pub, robot3_cmd_pub, robot3_vel_pub, robot4_cmd_pub, robot4_vel_pub, errors_pub,desired_traj_pub; // node publishers
std::vector<ros::Publisher> robots_cmd_pub;
tf::Point Odom_pos;    //odometry position (x,y)
double Odom_yaw;    //odometry orientation (yaw)
geometry_msgs::Pose2D qd,robot_pose1, robot_pose2, robot_pose3,robot_pose4,err;
geometry_msgs::Twist vel_msg, vel_msg3, vel_msg4;
double robot1_vel, robot2_vel, robot1_phi, robot2_phi, robot3_vel, robot3_omega;
std::vector<geometry_msgs::Pose2D> actual_pose;
std::vector<double> UpdateControlInputTwoWheel(int timestep, const std::vector<Pose>& path, const geometry_msgs::Pose2D& robot_pose, 
						double ev, double ew, ros::Publisher& robot_vel_pub,geometry_msgs::Twist& vel_msg);
void UpdateControlInputCar(int timestep, const Trajectory& traj, const geometry_msgs::Pose2D& robot_pose, 
						ros::Publisher& robot_vel_pub, geometry_msgs::Twist& vel_msg);
void zero_input(ros::Publisher& robot_vel_pub, geometry_msgs::Twist& vel_msg);

std::vector<Pose> generate_path(std::string filename) {
	std::vector<Pose> Path;
	Pose pose_;
	// 读取YAML文件
	YAML::Node path = YAML::LoadFile(filename);

	// 检查文件是否成功加载
	if (!path.IsDefined()) {
		std::cerr << "Failed to load yaml file.\n";
	}

	// 读取每个轨迹点
	for (const auto& kv : path) {
		const std::string& poseName = kv.first.as<std::string>();
		const YAML::Node& poseNode = kv.second;
		pose_.setX(poseNode["x"].as<double>());
		pose_.setY(poseNode["y"].as<double>());
		pose_.setTheta(poseNode["theta"].as<double>());
		Path.push_back(pose_);
	}
	return Path;
}

Trajectory_temp generate_traj(std::string filename) {
	Trajectory_temp traj;
	TrajectoryPoint_temp traj_point;
	// 读取YAML文件
	YAML::Node traj_ = YAML::LoadFile(filename);

	// 检查文件是否成功加载
	if (!traj_.IsDefined()) {
		std::cerr << "Failed to load yaml file.\n";
	}

	// 读取每个轨迹点
	for (const auto& kv : traj_) {
		const std::string& poseName = kv.first.as<std::string>();
		const YAML::Node& poseNode = kv.second;
		traj_point.x = poseNode["x"].as<double>();
		traj_point.y = poseNode["y"].as<double>();
		traj_point.theta = poseNode["theta"].as<double>();
		traj_point.phi = poseNode["phi"].as<double>();
		traj_point.v = poseNode["v"].as<double>();
		traj_point.omega = poseNode["omega"].as<double>();
		traj_point.a = poseNode["a"].as<double>();
		traj_point.t = poseNode["t"].as<double>();
		traj.push_back(traj_point);
	}
	return traj;
}

Trajectory_temp generate_ref_traj(Trajectory_temp traj_lead, double offset) {
	Trajectory_temp traj_follower; 
	traj_follower.resize(traj_lead.size());
	for (size_t l_index = 0; l_index < traj_lead.size(); l_index++) {
		traj_follower[l_index].x = traj_lead[l_index].x + offset * sin(traj_lead[l_index].theta);
		traj_follower[l_index].y = traj_lead[l_index].y - offset * cos(traj_lead[l_index].theta);
		traj_follower[l_index].theta = traj_lead[l_index].theta;
		traj_follower[l_index].v = (1 + offset * tan(traj_lead[l_index].phi) / 0.65) * traj_lead[l_index].v;
		traj_follower[l_index].phi = atan(0.65 * tan(traj_lead[l_index].phi) / (0.65 + offset * traj_lead[l_index].phi));	
		traj_follower[l_index].a = traj_lead[l_index].a;
		traj_follower[l_index].omega = traj_lead[l_index].omega;
		traj_follower[l_index].t = traj_lead[l_index].t;
	}
	return traj_follower;
}

Trajectory_temp generate_ref_traj_diff(Trajectory_temp traj_lead, double offset, double angle) {
	Trajectory_temp traj_follower_diff; 
	traj_follower_diff.resize(traj_lead.size());
	for (size_t l_index = 0; l_index < traj_lead.size() - 1; l_index++) {
		traj_follower_diff[l_index].x = traj_lead[l_index].x + offset * cos(traj_lead[l_index].theta - angle);
		traj_follower_diff[l_index].y = traj_lead[l_index].y + offset * sin(traj_lead[l_index].theta - angle);
		traj_follower_diff[l_index].theta = traj_lead[l_index].theta;
    // traj_follower_diff[l_index].theta = atan2(traj_follower_diff[l_index + 1].y - traj_follower_diff[l_index].y, traj_follower_diff[l_index + 1].x - traj_follower_diff[l_index].x);
		traj_follower_diff[l_index].t = traj_lead[l_index].t;
	}
	for (size_t l_index = 0; l_index < traj_lead.size() - 1; l_index++) {
    	double ds = hypot(traj_follower_diff[l_index + 1].x - traj_follower_diff[l_index].x, traj_follower_diff[l_index + 1].y - traj_follower_diff[l_index].y);
		traj_follower_diff[l_index].v = ds / (traj_follower_diff[l_index + 1].t - traj_follower_diff[l_index].t);
		traj_follower_diff[l_index].omega = (traj_lead[l_index + 1].theta - traj_lead[l_index].theta) / (traj_follower_diff[l_index + 1].t - traj_follower_diff[l_index].t);
	}
		for (size_t l_index = 0; l_index < traj_lead.size() - 1; l_index++) {
			traj_follower_diff[l_index].phi = (traj_lead[l_index + 1].omega - traj_lead[l_index].omega) / (traj_follower_diff[l_index + 1].t - traj_follower_diff[l_index].t);	
			traj_follower_diff[l_index].a = (traj_lead[l_index + 1].v - traj_lead[l_index].v) / (traj_follower_diff[l_index + 1].t - traj_follower_diff[l_index].t);
	}
	traj_follower_diff[traj_lead.size() - 1].x = traj_lead[traj_lead.size() - 1].x + offset * cos(traj_lead[traj_lead.size() - 1].theta - angle);
	traj_follower_diff[traj_lead.size() - 1].y = traj_lead[traj_lead.size() - 1].y + offset * sin(traj_lead[traj_lead.size() - 1].theta - angle);
	traj_follower_diff[traj_lead.size() - 1].theta = traj_lead[traj_lead.size() - 2].theta;
	traj_follower_diff[traj_lead.size() - 1].v = 0.0;
	traj_follower_diff[traj_lead.size() - 1].phi = 0.0;
	traj_follower_diff[traj_lead.size() - 1].a = 0.0;
	traj_follower_diff[traj_lead.size() - 1].omega = 0.0;
	traj_follower_diff[traj_lead.size() - 1].t = traj_lead[traj_lead.size() - 1].t;
		return traj_follower_diff;
}

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

void robot_odom1(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    for (size_t i = 0; i < msg->name.size(); i++) {
		std::string target_model = "hunter2_base1";
		if (msg->name[i] == target_model) {
			Odom_yaw = tf::getYaw(msg->pose[i].orientation);
			robot_pose1.x = msg->pose[i].position.x;
			robot_pose1.y = msg->pose[i].position.y;
			robot_pose1.theta = Odom_yaw;   
			break;
		}
	}
}

void robot_odom2(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    for (size_t i = 0; i < msg->name.size(); i++) {
		std::string target_model = "hunter2_base2";
		if (msg->name[i] == target_model) {
			Odom_yaw = tf::getYaw(msg->pose[i].orientation);
			robot_pose2.x = msg->pose[i].position.x;
			robot_pose2.y = msg->pose[i].position.y;
			robot_pose2.theta = Odom_yaw;   
			break;
		}
	}
}

void robot_actual_pose3(const geometry_msgs::Pose msg)
{
    tf::pointMsgToTF(msg.position, Odom_pos);
    Odom_yaw = tf::getYaw(msg.orientation);
	robot_pose3.x = msg.position.x;
	robot_pose3.y = msg.position.y;
    robot_pose3.theta = Odom_yaw;   
}

void robot_actual_pose4(const geometry_msgs::Pose msg)
{
    tf::pointMsgToTF(msg.position, Odom_pos);
    Odom_yaw = tf::getYaw(msg.orientation);
	robot_pose4.x = msg.position.x;
	robot_pose4.y = msg.position.y;
    robot_pose4.theta = Odom_yaw;   
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

std::vector<double> cartesian_controller_diff(const geometry_msgs::Pose2D& actual_pose, const double target_pose_x, const double target_pose_y, const double target_pose_theta,
										 const double target_vel, const double target_angvel) {
	std::vector<double> control_cmd;
	control_cmd.resize(2);
	double theta_act, e_x, e_y, e_local_x, e_local_y, u_w, u_v;
	theta_act = actual_pose.theta;
	e_x = target_pose_x - actual_pose.x;
	e_y = target_pose_y - actual_pose.y;
	e_local_x = cos(theta_act) * e_x + sin(theta_act) * e_y;
	e_local_y = -sin(theta_act) * e_x + cos(theta_act) * e_y;
	u_w = target_angvel + target_vel * (Ky * e_local_y + Ktheta * sin(target_pose_theta - theta_act));
	u_v = target_vel * cos(target_pose_theta - theta_act) + Kx * e_local_x;
	control_cmd[0] = u_v;
	control_cmd[1] = u_w;
	return control_cmd;
}

std::vector<double> cartesian_controller_car(const geometry_msgs::Pose2D& actual_pose, const double target_pose_x, const double target_pose_y, const double target_pose_theta,
										 const double target_vel, const double target_angvel) {
	std::vector<double> control_cmd;
	control_cmd.resize(2);
	double theta_act, e_x, e_y, e_local_x, e_local_y, u_w, u_v, e_theta;
	theta_act = actual_pose.theta;
	e_x = target_pose_x - actual_pose.x;
	e_y = target_pose_y - actual_pose.y;
	e_local_x = cos(theta_act) * e_x + sin(theta_act) * e_y;
	e_local_y = -sin(theta_act) * e_x + cos(theta_act) * e_y;
	e_theta = target_pose_theta - theta_act;
	u_v = K1 * e_local_x + target_vel * cos(e_theta);
	// u_w = atan(target_vel * tan(target_angvel) / u_v + K2 * wheel_base * e_local_y * target_vel / u_v + K3 * wheel_base * sin(e_theta) / u_v);
	u_w = atan((target_vel * tan(target_angvel) + K2 * e_local_y * wheel_base + K3 * e_theta * wheel_base) / u_v);
	control_cmd[0] = u_v;
	control_cmd[1] = u_w;
	return control_cmd;
}

std::vector<double> compute_tarj_length(const Trajectory_temp& traj) {
	std::vector<double> path_lengths = {0};
	for (int i = 1; i < traj.size(); i++) {
		path_lengths.push_back(path_lengths[i-1] + hypot(traj[i].x - traj[i-1].x, traj[i].y - traj[i-1].y));
	}
	return path_lengths;
}

bool receive_poses(std::vector<geometry_msgs::Pose2D> actual_poses) {
	bool received = true;
	for (int i = 0; i < actual_poses.size(); i++) {
		if (actual_poses[i].x == 0 && actual_poses[i].y == 0) {
			received = false;
		}
	}
	return received;
}

int main(int argc, char **argv)
{
	std::vector<double> x1_orig_plot, y1_orig_plot, theta1_orig_plot, x2_orig_plot, y2_orig_plot, theta2_orig_plot, x3_orig_plot, y3_orig_plot, theta3_orig_plot, x4_orig_plot, y4_orig_plot, theta4_orig_plot;
	std::vector<double> x1_plot, y1_plot, theta1_plot, x2_plot, y2_plot, theta2_plot, x3_plot, y3_plot, theta3_plot, x4_plot, y4_plot, theta4_plot;
	auto traj_1 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/ras_demo/formation_coordination/result/leader0.yaml");
	// auto traj_fol1 = generate_ref_traj(traj_1, 2.016733665);
	auto traj_fol_diff1 = generate_ref_traj_diff(traj_1, hypot(-18-(-20), -21.0083668325-(-20)), atan2(21.0083668325-20, 20-18));
	auto traj_fol1 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/ras_demo/formation_coordination/result/car_like0.yaml");
    std::vector<double> distances(3, 0.0), current_distances(3, 0.0), distances_old(3, 0.0), current_vels(3, 0.0), current_omegas(3, 0.0), current_thetas(3, 0.0);
	std::vector<double> target_pose_x(3, 0.0), target_pose_y(3, 0.0), target_pose_theta(3, 0.0), target_vels(3, 0.0), target_angvels(3, 0.0);
	ros::Time timestamp_old;
	ros::Duration dt;
	std::vector<std::vector<double>> path_lengths;
	int robot_num = 3;
	auto traj_fol_diff_r = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/ras_demo/formation_coordination/result/diff_drive0.yaml");
	// auto traj_fol_diff_r2 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/ras_demo/corridor_rec/traj_diff2.yaml");
	// std::vector<double> x, y;
	// for (int i = 0; i < traj_fol_diff_r.size(); i++) {
	// 	x.push_back(traj_fol_diff_r[i].t);
	// 	y.push_back(traj_fol_diff_r[i].phi);
	// }
	// plt::plot(x, y);
	// plt::show();
	std::vector<Trajectory_temp> traj_set;
	traj_set.resize(robot_num);
	traj_set[0] = traj_1; traj_set[1] = traj_fol1; traj_set[2] = traj_fol_diff_r; 
	// traj_set[3] = traj_fol_diff_r2;
	path_lengths.resize(robot_num);
	robots_cmd.resize(robot_num);
	int step_num = traj_1.size();
	for (int i = 0; i < traj_1.size(); i++) {
		x1_orig_plot.push_back(traj_1[i].x);
		y1_orig_plot.push_back(traj_1[i].y);
		theta1_orig_plot.push_back(traj_1[i].theta);
	}
	for (int i = 0; i < traj_fol1.size(); i++) {
		x2_orig_plot.push_back(traj_fol1[i].x);
		y2_orig_plot.push_back(traj_fol1[i].y);
		theta2_orig_plot.push_back(traj_fol1[i].theta);
	}
	for (int i = 0; i < traj_fol_diff1.size(); i++) {
		x3_orig_plot.push_back(traj_fol_diff1[i].x);
		y3_orig_plot.push_back(traj_fol_diff1[i].y);
		theta3_orig_plot.push_back(traj_fol_diff1[i].theta);
	}
	for (int i = 0; i < traj_fol_diff_r.size(); i++) {
		x4_orig_plot.push_back(traj_fol_diff_r[i].x);
		y4_orig_plot.push_back(traj_fol_diff_r[i].y);
		theta4_orig_plot.push_back(traj_fol_diff_r[i].theta);
	}
	for (int i = 0; i < robot_num; i++) {
		target_pose_x[i] = traj_set[i][0].x;
		target_pose_y[i] = traj_set[i][0].y;
		target_pose_theta[i] = traj_set[i][0].theta;
		current_thetas[i] = traj_set[i][0].theta;
		path_lengths[i] = compute_tarj_length(traj_set[i]);
	}
	auto obstacle = generate_obs("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/obs.yaml");
	// initialization of ROS node
    ros::init(argc, argv, "PID_trajectory_tracking_node");
    ros::NodeHandle n;
	ros::Subscriber sub_pose_robot1 = n.subscribe("gazebo/model_states", 1000, robot_odom1);
	ros::Subscriber sub_pose_robot2 = n.subscribe("gazebo/model_states", 1000, robot_odom2);
    ros::Subscriber sub_odometry_robot1 = n.subscribe("/mir1/mir_pose_simple", 1000 , robot_actual_pose3);
    // ros::Subscriber sub_odometry_robot2 = n.subscribe("/mir2/mir_pose_simple", 1000 , robot_actual_pose4);
	robot1_cmd_pub = n.advertise<geometry_msgs::Twist>("/hunter2_base1/ackermann_steering_controller/cmd_vel", 1000);
	robot2_cmd_pub = n.advertise<geometry_msgs::Twist>("/hunter2_base2/ackermann_steering_controller/cmd_vel", 1000);
	robot3_cmd_pub = n.advertise<geometry_msgs::Twist>("/mir1/mobile_base_controller/cmd_vel", 1000);
	// robot4_cmd_pub = n.advertise<geometry_msgs::Twist>("/mir2/mobile_base_controller/cmd_vel", 1000);	
	ros::Rate loop_rate(100);	
	int path_index = 1;
	int path_index_old = 0;
	actual_pose.resize(robot_num);
	robots_cmd_pub.resize(robot_num);
	robots_cmd_pub[0] = robot1_cmd_pub; robots_cmd_pub[1] = robot2_cmd_pub; robots_cmd_pub[2] = robot3_cmd_pub;
	timestamp_old = ros::Time::now(); 
	while(path_index < traj_set[0].size() - 2 && ros::ok()) {
		actual_pose[0] = robot_pose1; actual_pose[1] = robot_pose2; actual_pose[2] = robot_pose3;
		// compute distance to next point
		for (int i = 0; i < robot_num; i++) {
			distances[i] = path_lengths[i][path_index] - current_distances[i];
		}
		// compute target velocity
		for (int i = 0; i < robot_num; i++) {
			target_vels[i] = traj_set[i][path_index].v;
		}
		// target_vels = KP_vel * distances * control_rate;
		// check if next point is reached
		if (receive_poses(actual_pose)) {
			for (int i = 0; i < robot_num; i++) {
				if (distances[i] <= fabs(target_vels[i]) * loop_rate.expectedCycleTime().toSec()) {
					path_index += 1;
					distances_old[i]  = distances[i];
					break;
				}
				if (distances[i] > distances_old[i] && path_index != path_index_old) {
					path_index += 1;
					break;
				}
			}
		}
		if (hypot(robot_pose1.x - traj_set[0].back().x, robot_pose1.y - traj_set[0].back().y) < 0.5 && fabs(robot_pose1.theta - traj_set[0].back().theta) < 0.1 &&
		    hypot(robot_pose2.x - traj_set[1].back().x, robot_pose2.y - traj_set[1].back().y) < 0.5 && fabs(robot_pose2.theta - traj_set[1].back().theta) < 0.1 &&
		    hypot(robot_pose3.x - traj_set[2].back().x, robot_pose3.y - traj_set[2].back().y) < 0.5 && fabs(robot_pose3.theta - traj_set[2].back().theta) < 0.1) {
			break;
		}
		distances_old = distances;
		path_index_old = path_index;
		// compute next target point
		for (int i = 0; i < robot_num; i++) {
			target_pose_x[i] = traj_set[i][path_index].x;
			target_pose_y[i] = traj_set[i][path_index].y;
			target_pose_theta[i] = traj_set[i][path_index].theta;
			if (target_pose_theta[i] > M_PI) {
				target_pose_theta[i] -= 2 * M_PI;
			}
			else if (target_pose_theta[i] < -M_PI) {
				target_pose_theta[i] += 2 * M_PI;
			}
			target_angvels[i] = i < 2 ? traj_set[i][path_index].phi : traj_set[i][path_index].omega;
		}
		dt = ros::Time::now() - timestamp_old;
		for (int i = 0; i < robot_num; i++) {
			current_vels[i] = target_vels[i];
			current_omegas[i] = target_angvels[i];
			current_thetas[i] += target_angvels[i] * dt.toSec();
			current_distances[i] += fabs(target_vels[i]) * dt.toSec();
		}
		timestamp_old = ros::Time::now();
		if (receive_poses(actual_pose)) {
			for (int i = 0; i < robot_num; i++) {
				robots_cmd[i].linear.x = target_vels[i];
				robots_cmd[i].angular.z = target_angvels[i];
				auto cmd = i < 2 ? cartesian_controller_car(actual_pose[i], target_pose_x[i], target_pose_y[i], target_pose_theta[i], target_vels[i], target_angvels[i]) :
				cartesian_controller_diff(actual_pose[i], target_pose_x[i], target_pose_y[i], target_pose_theta[i], target_vels[i], target_angvels[i]);
				robots_cmd[i].linear.x = cmd[0];
				robots_cmd[i].angular.z = cmd[1];
				robots_cmd_pub[i].publish(robots_cmd[i]);
			}
		}
		else {
			for (int i = 0; i < robot_num; i++) {
				robots_cmd[i].linear.x = 0.0;
				robots_cmd[i].angular.z =0.0;			
				robots_cmd_pub[i].publish(robots_cmd[i]);
			}
		}
		if (robot_pose1.x != 0 && robot_pose1.y != 0 && robot_pose1.theta != 0) {
			x1_plot.push_back(robot_pose1.x);
			y1_plot.push_back(robot_pose1.y);
			theta1_plot.push_back(robot_pose1.theta);
		}
		if (robot_pose2.x != 0 && robot_pose2.y != 0 && robot_pose2.theta != 0) {
			x2_plot.push_back(robot_pose2.x);
			y2_plot.push_back(robot_pose2.y);
			theta2_plot.push_back(robot_pose2.theta);
		}
		if (robot_pose3.x != 0 && robot_pose3.y != 0 && robot_pose3.theta != 0) {
			x3_plot.push_back(robot_pose3.x);
			y3_plot.push_back(robot_pose3.y);
			theta3_plot.push_back(robot_pose3.theta);
		}
		// ROS_WARN("path index: %d", path_index);
		ros::spinOnce();
		loop_rate.sleep();
	};

	plt::clf();
	std::map<std::string, std::string> keywords;
	keywords["color"] = "black";
	for (int i = 0; i < x1_orig_plot.size(); i++) {
		plot_car(x1_orig_plot[i], y1_orig_plot[i], theta1_orig_plot[i], 1, i);
	}
	for (int i = 0; i < x1_plot.size(); i++) {
		plot_car(x1_plot[i], y1_plot[i], theta1_plot[i], 2, i);
	}
	// for (const auto& obs : obstacle) {
	// 	std::vector<double> obs_x, obs_y;
	// 	for (const auto& vert : obs) {
	// 		obs_x.push_back(vert.x() / 10);
	// 		obs_y.push_back(vert.y() / 10);
	// 	}
	// 	plt::fill(obs_x, obs_y, keywords);
	// }
	// plt::title("Car-like Robot1 Trajectory"); 
	// plt::figure();
	for (int i = 0; i < x2_orig_plot.size(); i++) {
		plot_car(x2_orig_plot[i], y2_orig_plot[i], theta2_orig_plot[i], 3, i);
	}
	for (int i = 0; i < x2_plot.size(); i++) {
		plot_car(x2_plot[i], y2_plot[i], theta2_plot[i], 4, i);
	}
	// for (const auto& obs : obstacle) {
	// 	std::vector<double> obs_x, obs_y;
	// 	for (const auto& vert : obs) {
	// 		obs_x.push_back(vert.x() / 10);
	// 		obs_y.push_back(vert.y() / 10);
	// 	}
	// 	plt::fill(obs_x, obs_y, keywords);
	// }
	// plt::title("Car-like Robot2 Trajectory"); 
	// plt::figure();
	for (int i = 0; i < x3_orig_plot.size(); i++) {
		plot_car(x4_orig_plot[i], y4_orig_plot[i], theta4_orig_plot[i], 1, i);
	}
	// for (i = 0; i < x1_orig_plot.size(); i++) {
	// 	plot_car(x3_plot[i] - 8, y3_plot[i] - 11.0083668325, theta3_plot[i], 2, i);
	// }
	for (int i = 0; i < x3_plot.size(); i++) {
		plot_car(x3_plot[i], y3_plot[i], theta3_plot[i], 4, i);
	}
	for (const auto& obs : obstacle) {
		std::vector<double> obs_x, obs_y;
		for (const auto& vert : obs) {
			obs_x.push_back(vert.x());
			obs_y.push_back(vert.y());
		}
		plt::fill(obs_x, obs_y, keywords);
	}
	// plt::title("Diff-drive Robot1 Trajectory"); 
	plt::title("Trajectory tracking result for triangle formation"); 
	plt::show();
    return 0;
}
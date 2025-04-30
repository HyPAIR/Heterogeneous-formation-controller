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
#include "yaml-cpp/yaml.h"
#include <vector>
#include "heterogeneous_formation_controller/math/pose.h"
#include "heterogeneous_formation_controller/optimizer_interface.h"
#include "traj_tracking/matplotlibcpp.h"
#include <ackermann_msgs/AckermannDriveStamped.h>

namespace plt = matplotlibcpp;
using namespace heterogeneous_formation_controller;
using namespace math;

// function to get robot position and orientation from odom topic
void plot_circle (const double& x, const double& y, int index, int pose_index);
void plot_car (const double& x, const double& y, const double& theta, int index, int pose_index);
std::vector<Pose> generate_path(std::string filename);
Trajectory_temp generate_traj(std::string filename);
Trajectory_temp generate_ref_traj(Trajectory_temp traj_lead, double offset);
Trajectory_temp generate_ref_traj_diff(Trajectory_temp traj_lead, double offset, double angle);
// function to calculate the euclidean distance between robot and desired traj 
double getDistance(double x1, double y1, double x2, double y2);
void plot_kinematic(std::vector<double> t, std::vector<double> v_car, std::vector<double> a_car, std::vector<double> phi_car, std::vector<double> omega_car);

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
		// traj_follower[l_index].v = (1 + offset * tan(traj_lead[l_index].phi) / 0.65) * traj_lead[l_index].v;
		traj_follower[l_index].v = traj_lead[l_index].v + traj_lead[l_index].v * tan(traj_lead[l_index].phi) * offset / 0.65;
		traj_follower[l_index].phi = atan(0.65 * tan(traj_lead[l_index].phi) / (0.65 + offset * traj_lead[l_index].phi));	
		traj_follower[l_index].t = traj_lead[l_index].t;
	}
	for (size_t l_index = 1; l_index < traj_lead.size(); l_index++) {
		traj_follower[l_index].a = (traj_lead[l_index].v - traj_lead[l_index - 1].v) / (traj_follower[l_index].t - traj_follower[l_index - 1].t);
		traj_follower[l_index].omega = (traj_lead[l_index].phi - traj_lead[l_index - 1].phi) / (traj_follower[l_index].t - traj_follower[l_index - 1].t);
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

// function to calculate the euclidean distance between robot and desired traj 
double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

void plot_kinematic(std::vector<double> t, std::vector<double> v_car, std::vector<double> a_car, std::vector<double> phi_car, std::vector<double> omega_car) {
	std::vector<double> v_max;
	for (int i = 0; i < v_car.size(); i++) {
		v_max.push_back(1.0);
	}
	plt::plot(t, v_car);
	plt::plot(t, v_max);
	plt::title("Car-like Robot Velocity"); 
	plt::figure();
	std::vector<double> a_max;
	for (int i = 0; i < a_car.size(); i++) {
		a_max.push_back(1.0);
	}
	plt::plot(t, a_car);
	plt::title("Car-like Robot Accleration"); 
	plt::figure();
	std::vector<double> phi_max;
	for (int i = 0; i < phi_car.size(); i++) {
		phi_max.push_back(0.62);
	}
	plt::plot(t, phi_car);
	plt::plot(t, phi_max);
	plt::title("Car-like Robot1 Steering Angle"); 
	plt::figure();
	std::vector<double> omega_max;
	for (int i = 0; i < omega_car.size(); i++) {
		std::vector<double> v_max;
		v_max.push_back(0.2);
	}
	plt::plot(t, omega_car);
	plt::plot(t, omega_max);
	plt::title("Car-like Robot Angular Velocity"); 
	plt::show();
}

void plot_v_diff(std::vector<double> t, std::vector<double> v_diff1) {
	std::vector<double> v_max;
	for (int i = 0; i < t.size(); i++) {
		v_max.push_back(1.0);
	}
    std::map<std::string, std::string> style1;      
	style1 = {{"color", "blue"}, {"label", "Diff-drive robot 1"}};
	plt::clf();
	plt::plot(t, v_diff1, style1);
	plt::plot(t, v_max, {{"color", "black"}});
	plt::xlabel("t");
	plt::ylabel("linear velocity");
	plt::legend();
	plt::show();
}

void plot_a_diff(std::vector<double> t, std::vector<double> a_diff1) {
	std::vector<double> a_max;
	for (int i = 0; i < t.size(); i++) {
		a_max.push_back(1.0);
	}
    std::map<std::string, std::string> style1;      
	style1 = {{"color", "blue"}, {"label", "Diff-drive robot 1"}};
	plt::clf();
	plt::plot(t, a_diff1, style1);
	plt::plot(t, a_max, {{"color", "black"}});
	plt::xlabel("t");
	plt::ylabel("acceleration");
	plt::legend();
	plt::show();
}

void plot_omega_diff(std::vector<double> t, std::vector<double> omega_diff1) {
	std::vector<double> omega_max;
	for (int i = 0; i < t.size(); i++) {
		omega_max.push_back(1.0);
	}
    std::map<std::string, std::string> style1, style2;      
	style1 = {{"color", "blue"}, {"label", "Diff-drive robot 1"}};
	plt::clf();
	plt::plot(t, omega_diff1, style1);
	plt::plot(t, omega_max, {{"color", "black"}});
	plt::xlabel("t");
	plt::ylabel("angular velocity");
	plt::legend();
	plt::show();
}

void plot_w_diff(std::vector<double> t, std::vector<double> w_diff1) {
	std::vector<double> w_max;
	for (int i = 0; i < t.size(); i++) {
		w_max.push_back(1.0);
	}
    std::map<std::string, std::string> style1;      
	style1 = {{"color", "blue"}, {"label", "Diff-drive robot 1"}};
	plt::clf();
	plt::plot(t, w_diff1, style1);
	plt::plot(t, w_max, {{"color", "black"}});
	plt::xlabel("t");
	plt::ylabel("angular acceleration");
	plt::legend();
	plt::show();
}

void plot_v_car(std::vector<double> t, std::vector<double> v_car1, std::vector<double> v_car2) {
	std::vector<double> v_max;
	for (int i = 0; i < t.size(); i++) {
		v_max.push_back(1.0);
	}
    std::map<std::string, std::string> style1, style2;      
	style1 = {{"color", "blue"}, {"label", "Car-like robot 1"}};
	style2 = {{"color", "red"}, {"label", "Car-like robot 2"}};
	plt::clf();
	plt::plot(t, v_car1, style1);
	plt::plot(t, v_car2, style2);
	plt::plot(t, v_max, {{"color", "black"}});
	plt::xlabel("t");
	plt::ylabel("linear velocity");
	plt::legend();
	plt::show();
}

void plot_a_car(std::vector<double> t, std::vector<double> a_car1, std::vector<double> a_car2) {
	std::vector<double> a_max;
	for (int i = 0; i < t.size(); i++) {
		a_max.push_back(1.0);
	}
    std::map<std::string, std::string> style1, style2;      
	style1 = {{"color", "blue"}, {"label", "Car-like robot 1"}};
	style2 = {{"color", "red"}, {"label", "Car-like robot 2"}};
	plt::clf();
	plt::plot(t, a_car1, style1);
	plt::plot(t, a_car2, style2);
	plt::plot(t, a_max, {{"color", "black"}});
	plt::xlabel("t");
	plt::ylabel("acceleration");
	plt::legend();
	plt::show();
}

void plot_omega_car(std::vector<double> t, std::vector<double> omega_car1, std::vector<double> omega_car2) {
	std::vector<double> omega_max;
	for (int i = 0; i < t.size(); i++) {
		omega_max.push_back(0.62);
	}
    std::map<std::string, std::string> style1, style2;      
	style1 = {{"color", "blue"}, {"label", "Car-like robot 1"}};
	style2 = {{"color", "red"}, {"label", "Car-like robot 2"}};
	plt::clf();
	plt::plot(t, omega_car1, style1);
	plt::plot(t, omega_car2, style2);
	plt::plot(t, omega_max, {{"color", "black"}});
	plt::xlabel("t");
	plt::ylabel("Steering angle");
	plt::legend();
	plt::show();
}

void plot_w_car(std::vector<double> t, std::vector<double> w_car1, std::vector<double> w_car2) {
	std::vector<double> w_max;
	for (int i = 0; i < t.size(); i++) {
		w_max.push_back(0.2);
	}
    std::map<std::string, std::string> style1, style2;      
	style1 = {{"color", "blue"}, {"label", "Car-like robot 1"}};
	style2 = {{"color", "red"}, {"label", "Car-like robot 2"}};
	plt::clf();
	plt::plot(t, w_car1, style1);
	plt::plot(t, w_car2, style2);
	plt::plot(t, w_max, {{"color", "black"}});
	plt::xlabel("t");
	plt::ylabel("angular velocity");
	plt::legend();
	plt::show();
}

void plot_formation_linear_error(std::vector<double> t, Trajectory traj_1, Trajectory traj_fol1, Trajectory traj_fol_diff_r1) {
	std::vector<double> error_12, error_13;
	for (int i = 0; i < traj_1.size(); i++) {
		error_12.push_back(hypot(traj_1[i].x - traj_fol1[i].x, traj_1[i].y - traj_fol1[i].y) - 2.0);
		error_13.push_back(hypot(traj_1[i].x - traj_fol_diff_r1[i].x, traj_1[i].y - traj_fol_diff_r1[i].y) - hypot(-18-(-20), -21.0083668325-(-20)));
	}
    std::map<std::string, std::string> style1, style2, style3;      
	style1 = {{"color", "blue"}, {"label", "Diff-drive robot 1"}};
	style2 = {{"color", "red"}, {"label", "Car-like robot 1"}};
	plt::clf();
	plt::plot(t, error_12, style1);
	plt::plot(t, error_13, style2);
	plt::xlabel("t");
	plt::ylabel("linear error");
	plt::legend();
	plt::show();
	plt::clf();	
}

void plot_formation_angular_error(std::vector<double> t, Trajectory traj_1, Trajectory traj_fol1, Trajectory traj_fol_diff_r1) {
	std::vector<double> error_12, error_13, error_14;
	for (int i = 0; i < traj_1.size(); i++) {
		error_12.push_back(traj_1[i].theta - traj_fol1[i].theta);
		error_13.push_back(traj_1[i].theta - traj_fol_diff_r1[i].theta);
	}
    std::map<std::string, std::string> style1, style2, style3;      
	style1 = {{"color", "blue"}, {"label", "Diff-drive robot 1"}};
	style2 = {{"color", "red"}, {"label", "Car-like robot 1"}};
	plt::clf();
	plt::plot(t, error_12, style1);
	plt::plot(t, error_13, style2);
	plt::xlabel("t");
	plt::ylabel("angular error");
	plt::legend();
	plt::show();
	plt::clf();	
}

int main(int argc, char **argv)
{
	std::vector<double> x1_orig_plot, y1_orig_plot, theta1_orig_plot, x2_orig_plot, y2_orig_plot, theta2_orig_plot, x3_orig_plot, y3_orig_plot, theta3_orig_plot, x4_orig_plot, y4_orig_plot, theta4_orig_plot;
	std::vector<double> x1_plot, y1_plot, theta1_plot, x2_plot, y2_plot, theta2_plot, x3_plot, y3_plot, theta3_plot, x4_plot, y4_plot, theta4_plot;
	auto traj_1 = generate_traj("/home/weijian/hmfpc_test_ws/src/heterogeneous_formation_controller/traj_test_3.yaml");
	auto traj_fol1 = generate_ref_traj(traj_1, 2.016733665);
	// auto traj_fol1 = generate_ref_traj(traj_1, 1.8);
	auto traj_fol_diff1 = generate_ref_traj_diff(traj_1, hypot(-18-(-20), -21.0083668325-(-20)), atan2(21.0083668325-20, 20-18));
	std::vector<double> t_set, v_diff1, w_diff1, a_diff1, omega_diff1, v_diff2, w_diff2, a_diff2, omega_diff2, v_car1, phi_car1, a_car1, omega_car1, v_car2, phi_car2, a_car2, omega_car2;
	auto traj_fol_diff_r1 = generate_traj("/home/weijian/hmfpc_test_ws/src/heterogeneous_formation_controller/traj_test_diff3_1.yaml");
	for (int i = 0; i < traj_1.size(); i++) {
		t_set.push_back(traj_1[i].t);
		v_car1.push_back(traj_1[i].v);
		a_car1.push_back(traj_1[i].a);
		phi_car1.push_back(traj_1[i].phi);
		omega_car1.push_back(traj_1[i].omega);
	}
	for (int i = 0; i < traj_fol1.size(); i++) {
		v_car2.push_back(traj_fol1[i].v);
		a_car2.push_back(traj_fol1[i].a);
		phi_car2.push_back(traj_fol1[i].phi);
		omega_car2.push_back(traj_fol1[i].omega);
	}
	for (int i = 0; i < traj_fol_diff_r1.size(); i++) {
		v_diff1.push_back(traj_fol_diff_r1[i].v);
		a_diff1.push_back(traj_fol_diff_r1[i].a);
		w_diff1.push_back(traj_fol_diff_r1[i].phi);
		omega_diff1.push_back(traj_fol_diff_r1[i].omega);
	}
    // plot_v_diff(t_set, v_diff1);
    // plot_a_diff(t_set, a_diff1);
    // plot_omega_diff(t_set, omega_diff1);
    // plot_w_diff(t_set, w_diff1);
    // plot_v_car(t_set, v_car1, v_car2);
    // plot_a_car(t_set, a_car1, a_car2);
	// plot_omega_car(t_set, phi_car1, phi_car2);
	// plot_omega_car(t_set, omega_car1, omega_car2);

	// plot_formation_linear_error(t_set, traj_1, traj_fol1, traj_fol_diff_r1);
	// plot_formation_angular_error(t_set, traj_1, traj_fol1, traj_fol_diff_r1);

	// plot_kinematic(t_set, v_car1, a_car1, phi_car1, omega_car1);
	// plot_kinematic(t_set, v_car2, a_car2, phi_car2, omega_car2);
	// plot_kinematic(t_set, v_diff1, a_diff1, w_diff1, w_diff1);
	// plot_kinematic(t_set, v_diff2, a_diff2, w_diff2, w_diff2);
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
	for (int i = 0; i < traj_fol_diff_r1.size(); i++) {
		x3_orig_plot.push_back(traj_fol_diff_r1[i].x);
		y3_orig_plot.push_back(traj_fol_diff_r1[i].y);
		theta3_orig_plot.push_back(traj_fol_diff_r1[i].theta);
	}
	auto obstacle = generate_obs("/home/weijian/traj_tracking/src/obstacle.yaml");
	// initialization of ROS node
	int i = 0;
	int i_ = 0;
	plt::clf();
	std::map<std::string, std::string> keywords;
	keywords["color"] = "black";
	for (i = 0; i < x1_orig_plot.size(); i++) {
		plot_car(x1_orig_plot[i], y1_orig_plot[i], theta1_orig_plot[i], 1, i);
	}
	for (i = 0; i < x2_orig_plot.size(); i++) {
		plot_car(x2_orig_plot[i], y2_orig_plot[i], theta2_orig_plot[i], 1, i);
	}
	for (i = 0; i < x3_orig_plot.size(); i++) {
		plot_car(x3_orig_plot[i], y3_orig_plot[i], theta3_orig_plot[i], 2, i);
	}
	// for (const auto& obs : obstacle) {
	// 	std::vector<double> obs_x, obs_y;
	// 	for (const auto& vert : obs) {
	// 		obs_x.push_back(vert.x() / 10);
	// 		obs_y.push_back(vert.y() / 10);
	// 	}
	// 	plt::fill(obs_x, obs_y, keywords);
	// }
	plt::title("Optimal Trajectories"); 
	plt::show();
    return 0;
}
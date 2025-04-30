// Created by weijian on 17/11/23.
#include <iostream>
#include <ros/ros.h>
#include "task_assignment/Hungarian.h"
#include <formation_generation/param.hpp>
#include <formation_generation/mission.hpp>
#include "heterogeneous_formation_controller/heterogeneous_formation_controller.h"
#include "heterogeneous_formation_controller/math/generate_obs.h"
#include "heterogeneous_formation_controller/visualization/plot.h"
#include "heterogeneous_formation_controller/yaml_all.h"

using namespace heterogeneous_formation_controller;

void PlotPose1(std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config, int robot_index, int time_step, Trajectory_temp traj, std::vector<double> robot_type) {
    const std::string robot_name = "Footprint_" + std::to_string(robot_index);
    // visualization::Delete(i, robot_name);
    // visualization::Trigger();
    double theta = traj[time_step].theta;
    if (theta < -M_PI) {
        while (theta < -M_PI) {
            theta += 2 * M_PI;
        }
    }
    else if (theta > M_PI) {
        while (theta > M_PI) {
            theta -= 2 * M_PI;
        }
    }
    math::Pose robot_pos(traj[time_step].x, traj[time_step].y, traj[time_step].theta);
    auto box = config->vehicle.GenerateBox(robot_pos);
    auto color = robot_index > 7 ? visualization::Color::Green : visualization::Color::Cyan;
    // color.set_alpha(0.4);
    visualization::PlotPolygon(math::Polygon2d(box), 0.3, color, 1, robot_name);
    visualization::Trigger();
    // visualization::Plot(xs, ys, 0.5, visualization::Color::Magenta, i, robot_name_ta);
}

void PlotPose(std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config, int robot_index, int time_step, vector<vector<vector<double>>> plan, vector<double> robot_type) {
    const std::string robot_name = "Footprint_" + std::to_string(robot_index);
    // visualization::Delete(i, robot_name);
    // visualization::Trigger();
    double theta = plan[robot_index][time_step][2];
    if (theta < -M_PI) {
        while (theta < -M_PI) {
            theta += 2 * M_PI;
        }
    }
    else if (theta > M_PI) {
        while (theta > M_PI) {
            theta -= 2 * M_PI;
        }
    }
    math::Pose robot_pos(plan[robot_index][time_step][0], plan[robot_index][time_step][1], theta);
    auto box = config->vehicle.GenerateBox(robot_pos);
    auto color = robot_index > 7 ? visualization::Color::Green : visualization::Color::Cyan;
    // color.set_alpha(0.4);
    visualization::PlotPolygon(math::Polygon2d(box), 0.3, color, 1, robot_name);
    visualization::Trigger();
    // visualization::Plot(xs, ys, 0.5, visualization::Color::Magenta, i, robot_name_ta);
}

int main(int argc, char* argv[])
{
    auto plan =  generate_traj_fg("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/ras_demo/formation_generation/fg_16.yaml", 16);
    auto config_ = std::make_shared<PlannerConfig>();
    ros::init (argc, argv, "swarm_traj_planner_rbp");
    ros::NodeHandle nh( "~" );
    ros::Rate rate(40);
    int j = 0;
    SwarmPlanning::Mission mission;
	vector<vector<double>> diff_start, diff_goal;
	vector<vector<double>> car_start, car_goal;
    visualization::Init(nh, "odom", "/hmfpc_test_vis");
	math::GenerateObstacle generateobs;
  	std::vector<math::Pose> obstacle;
	std::vector<math::Polygon2d> polys_orig, polys;
	std::vector<std::vector<math::Vec2d>> poly_vertices_set;

	obstacle.push_back({-30, 30, 0.1});
	obstacle.push_back({-15, 30, 0.8});
	obstacle.push_back({0, 30, 0.3});
	obstacle.push_back({15, 30, 0.56});
	obstacle.push_back({30, 30, 0.26});
	obstacle.push_back({-30, -30, 0.75});
	obstacle.push_back({-15, -30, 0.83});
	obstacle.push_back({0, -30, 0.34});
	obstacle.push_back({15, -30, 0.2});
	obstacle.push_back({30, -30, 0.98});
	obstacle.push_back({-30, 15, 0.25});
	obstacle.push_back({-15, 15, 0.34});
	obstacle.push_back({0, 15, 0.63});
	obstacle.push_back({15, 15, 0.45});
	obstacle.push_back({30, 15, 0.72});
	obstacle.push_back({-30, -15, 0.23});
	obstacle.push_back({-12, -15, 0.62});
	obstacle.push_back({0, -15, 0.27});
	obstacle.push_back({15, -15, 0.86});
	obstacle.push_back({30, -15, 0.56});
	obstacle.push_back({22.5, 0, 0.25});
	obstacle.push_back({-22.5, 0, 0.89});
	// poly_vertices_set.push_back({{-30.5, -30}, {-30.5, 30}, {-30, 30}, {-30, -30}});
	// poly_vertices_set.push_back({{-30, 30}, {-30, 30.5}, {30, 30.5}, {30, 30}});
	// poly_vertices_set.push_back({{30, 30}, {30.5, 30}, {30.5, -30}, {30, -30}});
	// poly_vertices_set.push_back({{-30, -30}, {-30, -30.5}, {30, -30.5}, {30, -30}});

	polys.push_back(math::Polygon2d({{-30.5, -30}, {-30.5, 30}, {-30, 30}, {-30, -30}}));
	polys.push_back(math::Polygon2d({{-30, 30}, {-30, 30.5}, {30, 30.5}, {30, 30}}));
	polys.push_back(math::Polygon2d({{30, 30}, {30.5, 30}, {30.5, -30}, {30, -30}}));
	polys.push_back(math::Polygon2d({{-30, -30}, {-30, -30.5}, {30, -30.5}, {30, -30}}));
	// obstacle.push_back({0, 0, 0});
	// obstacle.push_back({0, 17, 0});
	// obstacle.push_back({0, -17, 0});
	for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
		math::Vec2d centre(obstacle[obs_ind].x(), obstacle[obs_ind].y());
		std::vector<math::Vec2d> poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), false);
		poly_vertices_set.push_back(poly_vertices);
		// polys_orig.push_back(math::Polygon2d(poly_vertices));
		polys.push_back(math::Polygon2d(poly_vertices));
	} 
    if(!mission.setMission(nh)){
        return -1;
    }
    while (ros::ok()) {
        // for (int i = 0; i < 4; i++) {
        // visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Grey, i, "Boundary"+  std::to_string(i));
        // }
        for (int i = 4; i < polys.size(); i++) {
        visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Red, i, "Obstacle"+  std::to_string(i));
        }
		for (int i = 0; i < plan.size(); i++) {
            PlotPose(config_, i, j, plan, mission.quad_type);
		}
  		visualization::Trigger();
        j++;
        if (j == plan[0].size()) {
            break;
        }
        ros::spinOnce();
        rate.sleep();	
	}

	return 0;
}

// ps aux | grep ros |  awk '{print $2}' | xargs kill -9; ps aux | grep rviz |  awk '{print $2}' | xargs kill -9

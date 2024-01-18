/**
 * file ta_test_node.cpp
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief task allocation test for heterogeneous formation
 * data 2023-11-22
 * 
 * @copyright Copyroght(c) 2023
*/
#include <iostream>
#include <ros/ros.h>
#include "task_assignment/Hungarian.h"
#include <formation_generation/param.hpp>
#include <formation_generation/mission.hpp>
#include "formation_planner/formation_planner.h"
#include "formation_planner/math/generate_obs.h"
#include "formation_planner/visualization/plot.h"

using namespace formation_planner;

int main(int argc, char* argv[])
{
    auto config_ = std::make_shared<PlannerConfig>();
    ros::init (argc, argv, "swarm_traj_planner_rbp");
    ros::NodeHandle nh( "~" );
    ros::Rate rate(20);
    SwarmPlanning::Mission mission;
	vector<vector<double>> diff_start, diff_goal;
	vector<vector<double>> car_start, car_goal;
    visualization::Init(nh, "map", "/ta_test_vis");
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
	obstacle.push_back({-15, -15, 0.62});
	obstacle.push_back({0, -15, 0.27});
	obstacle.push_back({15, -15, 0.86});
	obstacle.push_back({30, -15, 0.56});
	obstacle.push_back({22.5, 0, 0.25});
	obstacle.push_back({-22.5, 0, 0.89});
//   polys.push_back(math::Polygon2d({{-33 + 0.3, 28.8 + 0.3}, {-33 - 0.3, 28.8 + 0.3}, {-33 - 0.3, 28.8 - 0.3}, {-33 + 0.3, 28.8 - 0.3}}));
//   polys.push_back(math::Polygon2d({{-33 + 0.3, 21 + 0.3}, {-33 - 0.3, 21 + 0.3}, {-33 - 0.3, 21 - 0.3}, {-33 + 0.3, 21 - 0.3}}));
//   polys.push_back(math::Polygon2d({{-20 + 0.3, 28.8 + 0.3}, {-20 - 0.3, 28.8 + 0.3}, {-20 - 0.3, 28.8 - 0.3}, {-20 + 0.3, 28.8 - 0.3}}));
//   polys.push_back(math::Polygon2d({{-20 + 0.3, 21 + 0.3}, {-20 - 0.3, 21 + 0.3}, {-20 - 0.3, 21 - 0.3}, {-20 + 0.3, 21 - 0.3}}));
//    polys.push_back(math::Polygon2d({{40 + 1.0, 34 + 1.0}, {40 - 1.0, 34 + 1.0}, {40 - 1.0, 34 - 1.0}, {40 + 1.0, 34 - 1.0}}));
//   polys.push_back(math::Polygon2d({{58 + 1.0, 33.6 + 1.0}, {58 - 1.0, 33.6 + 1.0}, {58 - 1.0, 33.6 - 1.0}, {58+ 1.0, 33.6 - 1.0}}));
//   polys.push_back(math::Polygon2d({{40 + 1.0, 20.6 + 1.0}, {40 - 1.0, 20.6 + 1.0}, {40 - 1.0, 20.6 - 1.0}, {40 + 1.0, 20.6 - 1.0}}));
//   polys.push_back(math::Polygon2d({{58 + 1.0, 20.5 + 1.0}, {58 - 1.0, 20.5 + 1.0}, {58 - 1.0, 20.6 - 1.0}, {58 + 1.0, 20.6 - 1.0}}));
//   polys.push_back(math::Polygon2d({{ -43.7, 52.9}, {-74.3, 52.7}, {-73.7, 45.2}, {-45.2, 45.5}}));
//   polys.push_back(math::Polygon2d({{ -74, 52.2}, {-81.2, 52.5}, {-74.3, 10}, {-81.2, 10}})); 
//   polys.push_back(math::Polygon2d({{ -74, 10}, {-73.6, 16.1}, {-51.3, 22.1}, {-51, 10.7}})); 
//   polys.push_back(math::Polygon2d({{ -49.7, 10}, {-53.1, 10}, {-52.3, 0}, {-51, 0}})); 
//   polys.push_back(math::Polygon2d({{ -49.7, 0}, {-49.1,  -2}, {75, -2}, {75, 0}})); 
//   polys.push_back(math::Polygon2d({{ -42.6, 51.3}, {-1.3,  51.1}, {-1.6, 56}, {-46.1,  56}})); 
//   polys.push_back(math::Polygon2d({{ -0.65, 52.5}, {-1.3,  8.75}, {24.2, 8}, {23.6,  51.2}})); 
//   polys.push_back(math::Polygon2d({{ 39.5 + 1, 20 + 1}, {39.5 - 1,  20 + 1}, {39.5 - 1, 20 - 1}, {39.5 + 1,  20 - 1}})); 
//   polys.push_back(math::Polygon2d({{ 72.5, -1.5}, {76.7,  -1.5}, {76.7, 50}, {72.5,  50}})); 
	// poly_vertices_set.push_back({{-30.5, -30}, {-30.5, 30}, {-30, 30}, {-30, -30}});
	// poly_vertices_set.push_back({{-30, 30}, {-30, 30.5}, {30, 30.5}, {30, 30}});
	// poly_vertices_set.push_back({{30, 30}, {30.5, 30}, {30.5, -30}, {30, -30}});
	// poly_vertices_set.push_back({{-30, -30}, {-30, -30.5}, {30, -30.5}, {30, -30}});

	// polys.push_back(math::Polygon2d({{-30.5, -30}, {-30.5, 30}, {-30, 30}, {-30, -30}}));
	// polys.push_back(math::Polygon2d({{-30, 30}, {-30, 30.5}, {30, 30.5}, {30, 30}}));
	// polys.push_back(math::Polygon2d({{30, 30}, {30.5, 30}, {30.5, -30}, {30, -30}}));
	// polys.push_back(math::Polygon2d({{-30, -30}, {-30, -30.5}, {30, -30.5}, {30, -30}}));
	// obstacle.push_back({0, 0, 0});
	// obstacle.push_back({0, 17, 0});
	// obstacle.push_back({0, -17, 0});
	// obstacle.push_back({50, -45, 2.7});
	// obstacle.push_back({60, 20, 0});
	// obstacle.push_back({-40, -50, 2.3});
	// obstacle.push_back({60, 80, 2.5});
	// obstacle.push_back({-70, 70, 0});
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
	for (int i = 0; i < mission.qn; i++) {
		if (mission.quad_type[i] == 0) {
			diff_start.push_back(mission.startState[i]);
			diff_goal.push_back(mission.goalState[i]);
		}
		else {
			car_start.push_back(mission.startState[i]);
			car_goal.push_back(mission.goalState[i]);
		}
	}
	vector<vector<double>> costDiff(diff_start.size(), vector<double>(diff_goal.size())); 
	vector<vector<double>> costCar(car_start.size(), vector<double>(car_goal.size())); 
	for (int i = 0; i < diff_start.size(); i++) {
		for (int j = 0; j < diff_goal.size(); j++) {
			costDiff[i][j] = hypot(diff_start[i][0] - diff_goal[j][0], diff_start[i][1] - diff_goal[j][1]);
		}
	}
	for (int i = 0; i < car_start.size(); i++) {
		for (int j = 0; j < car_goal.size(); j++) {
			costCar[i][j] = hypot(car_start[i][0] - car_goal[j][0], car_start[i][1] - car_goal[j][1]);
		}
	}
    // please use "-std=c++11" for this initialization of vector.

	HungarianAlgorithm HungAlgo;
	vector<int> assignment_diff, assignment_car;

	double cost_diff = HungAlgo.Solve(costDiff, assignment_diff);
	std::cout << "Diff: " << std::endl;
	for (unsigned int x = 0; x < costDiff.size(); x++)
		std::cout << x << "," << assignment_diff[x] << "\t";

	std::cout << "\ncost: " << cost_diff << std::endl;

	double cost_car = HungAlgo.Solve(costCar, assignment_car);
	std::cout << "Car: " << std::endl;
	for (unsigned int x = 0; x < costCar.size(); x++)
		std::cout << x << "," << assignment_car[x] << "\t";

	std::cout << "\ncost: " << cost_diff << std::endl;
    while (ros::ok()) {
		for (int i = 0; i < polys.size(); i++) {
			visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Red, i, "Obstacle"+  std::to_string(i));
			visualization::Trigger();   
		}
		for (int i = 0; i < mission.qn; i++) {
			math::Pose start(mission.startState[i][0], mission.startState[i][1], 0.0);
			math::Pose goal(mission.goalState[i][0], mission.goalState[i][1], 0.0);
			std::vector<double> xs = {mission.startState[i][0], mission.goalState[i][0]};
			std::vector<double> ys = {mission.startState[i][1], mission.goalState[i][1]};
			auto box_s = config_->vehicle.GenerateBox(start);
			auto box_g = config_->vehicle.GenerateBox(goal);
			auto color = mission.quad_type[i] > 0 ? visualization::Color::Yellow : visualization::Color::White;
			color.set_alpha(0.4);
			const std::string robot_name_ta = "TaskAssignment" + std::to_string(i);
			const std::string robot_name_s = "Footprint_s_" + std::to_string(i);
			const std::string robot_name_g = "Footprint_g_" + std::to_string(i);
			visualization::PlotPolygon(math::Polygon2d(box_s), 0.5, color, i, robot_name_s);
			visualization::PlotPolygon(math::Polygon2d(box_g), 0.5, color, i, robot_name_g);
			color = mission.quad_type[i] > 0 ? visualization::Color::Magenta : visualization::Color::Green;
  			visualization::Plot(xs, ys, 0.2, color, i, robot_name_ta);
		}
  		visualization::Trigger();
        ros::spinOnce();
        rate.sleep();	
	}

	return 0;
}

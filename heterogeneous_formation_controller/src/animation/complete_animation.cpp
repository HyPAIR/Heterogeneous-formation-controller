/**
 * file complete_animation.cpp
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief complete animation 
 * data 2023-11-22
 * 
 * @copyright Copyroght(c) 2023
*/
#include <iostream>
#include <ros/ros.h>
#include "task_assignment/Hungarian.h"
#include <formation_generation/param.hpp>
#include <formation_generation/mission.hpp>
#include "heterogeneous_formation_controller/heterogeneous_formation_controller.h"
#include "heterogeneous_formation_controller/math/generate_obs.h"
#include "heterogeneous_formation_controller/visualization/plot.h"
#include "heterogeneous_formation_controller/yaml_all.h"
#include <nav_msgs/Path.h>

using namespace heterogeneous_formation_controller;

void DrawTrajectoryRviz(const FullStates sol_traj, std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config,
      int robot_index, ros::Publisher path_pub) {
  for (int i = 0; i < sol_traj.states.size(); i++) {
    auto x0_disc = config->vehicle.GetVertexPositions(sol_traj.states[i].x, sol_traj.states[i].y, sol_traj.states[i].theta, 1.0);
    std::vector<double> x1, y1;
    x1.push_back(x0_disc[0]);x1.push_back(x0_disc[2]);x1.push_back(x0_disc[4]);x1.push_back(x0_disc[6]);x1.push_back(x0_disc[0]);
    y1.push_back(x0_disc[1]);y1.push_back(x0_disc[3]);y1.push_back(x0_disc[5]);y1.push_back(x0_disc[7]);y1.push_back(x0_disc[1]);
    auto color = visualization::Color::White;
    color.set_alpha(0.03);
    visualization::Plot(x1, y1, 0.2, color, i, "spatial_envelopes" + std::to_string(robot_index));
  }
//   const std::string robot_name = "Footprint_" + std::to_string(robot_index);
//   for(int i = 0; i < 1e3; i++) {
//     visualization::Delete(i, robot_name);
//   }
//   visualization::Trigger();

//   nav_msgs::Path msg;
//   msg.header.frame_id = "map";
//   msg.header.stamp = ros::Time::now();
//   for(size_t i = 0; i < sol_traj.states.size(); i++) {
//     geometry_msgs::PoseStamped pose;
//     pose.header = msg.header;
//     pose.pose.position.x = sol_traj.states[i].x;
//     pose.pose.position.y = sol_traj.states[i].y;
//     pose.pose.orientation = tf::createQuaternionMsgFromYaw(sol_traj.states[i].theta);
//     msg.poses.push_back(pose);

//     auto box = config->vehicle.GenerateBox(sol_traj.states[i].pose());
//     auto color = robot_index > 0 ? visualization::Color::Yellow : visualization::Color::White;
//     color.set_alpha(0.4);
//     visualization::PlotPolygon(math::Polygon2d(box), 0.05, color, i, robot_name);
//   }
//   path_pub.publish(msg);
  visualization::Trigger();
}

FullStates traj2fullstates(Trajectory_temp traj) {
    FullStates solution;
    solution.states.resize(traj.size());
    solution.tf = traj[traj.size() - 1].t;
    for (int i = 0; i < traj.size(); i++) {
        solution.states[i].x = traj[i].x;
        solution.states[i].y = traj[i].y;
        solution.states[i].theta = traj[i].theta;
        solution.states[i].v = traj[i].v;
        solution.states[i].phi = traj[i].phi;
        solution.states[i].a = traj[i].a;
        solution.states[i].omega = traj[i].omega;
    }
    return solution;
}

void PlotPoseFG(std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config, int robot_index, int time_step, vector<vector<vector<double>>> plan, vector<double> robot_type) {
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
    auto color = robot_type[robot_index] > 0 ? visualization::Color::Green : visualization::Color::Cyan;
    visualization::PlotPolygon(math::Polygon2d(box), 0.3, color, 1, robot_name);
    visualization::Trigger();
    // visualization::Plot(xs, ys, 0.5, visualization::Color::Magenta, i, robot_name_ta);
}

void PlotTrajectoryFG(std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config, const std::vector<std::vector<std::vector<double>>>& traj_set) {
    for (int i = 0; i < traj_set.size(); i++) {
        const std::string robot_name = "FGTraj_" + std::to_string(i);
        for (int j = 0; j < traj_set[i].size(); j++) {
            math::Pose robot_pos(traj_set[i][j][0], traj_set[i][j][1],traj_set[i][j][2]);
            auto box = config->vehicle.GenerateBox(robot_pos);
            auto color = visualization::Color::Yellow;
            color.set_alpha(0.03);
            // color.set_alpha(0.4);
            visualization::PlotPolygon(math::Polygon2d(box), 0.3, color, j, robot_name);
            visualization::Trigger();           
        }
    }
}

void PlotPose(std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config, int robot_index, int time_step, Trajectory_temp traj, std::vector<double> robot_type) {
    if (time_step > traj.size() - 1) {
        time_step = traj.size() - 1;
    }
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
    auto color = robot_type[robot_index] > 0 ? visualization::Color::Green : visualization::Color::Cyan;
    // color.set_alpha(0.4);
    visualization::PlotPolygon(math::Polygon2d(box), 0.3, color, 1, robot_name);
    visualization::Trigger();
    // visualization::Plot(xs, ys, 0.5, visualization::Color::Magenta, i, robot_name_ta);
}
int main(int argc, char* argv[])
{
	std::vector<Trajectory_temp> traj_set, traj_set_leader;
    std::vector<FullStates> solution_set;
    FullStates solution1, solution2, solution3, solution4, solution5;
    std::string file_front = "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/complete_demo_2/result/";
    std::string file_back = ".yaml";
    // auto traj_1 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_oppositing/replan.yaml");
	// auto traj_fol1 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_oppositing/replan_car.yaml");
	// auto traj_fol_diff_r = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_oppositing/replan_diff.yaml");
    // auto traj_1_h = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_oppositing/leader_h.yaml");
	// auto traj_fol1_h = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_oppositing/solution_car_like_h.yaml");
	// auto traj_fol_diff_r_h = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_oppositing/solution_diff_drive_h.yaml");
    // auto traj_1 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_following/replan.yaml");
	// auto traj_fol1 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_following/replan_car.yaml");
	// auto traj_fol_diff_r = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_following/replan_diff.yaml");
    // auto traj_1_h = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_following/leader_h.yaml");
	// auto traj_fol1_h = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_following/solution_car_like_h.yaml");
	// auto traj_fol_diff_r_h = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_following/solution_diff_drive_h.yaml");
    auto traj_1 = generate_traj(file_front + "leader0.yaml");
    traj_set_leader.push_back(traj_1);
	auto traj_fol1 = generate_traj(file_front + "diff_drive0.yaml");
	auto traj_fol_diff_r1 = generate_traj(file_front + "car_like0.yaml");
    traj_set.push_back(traj_1); traj_set.push_back(traj_fol1); traj_set.push_back(traj_fol_diff_r1);
    solution1 = traj2fullstates(traj_1);
    solution_set.push_back(solution1);  
    auto traj_2 = generate_traj(file_front + "leader1.yaml");
    traj_set_leader.push_back(traj_2);
	auto traj_fol2 = generate_traj(file_front + "diff_drive1.yaml");
	auto traj_fol_diff_r21 = generate_traj(file_front + "diff_drive11.yaml");
	auto traj_fol_diff_r2 = generate_traj(file_front + "car_like1.yaml");
    traj_set.push_back(traj_2); traj_set.push_back(traj_fol2); traj_set.push_back(traj_fol_diff_r2);
    traj_set.push_back(traj_fol_diff_r21);
    solution2 = traj2fullstates(traj_2);
    solution_set.push_back(solution2);  
    auto traj_3 = generate_traj(file_front + "leader2.yaml");
    traj_set_leader.push_back(traj_3);
	auto traj_fol3 = generate_traj(file_front + "diff_drive2.yaml");
	auto traj_fol_diff_r3 = generate_traj(file_front + "car_like2.yaml");
    traj_set.push_back(traj_3); traj_set.push_back(traj_fol3); traj_set.push_back(traj_fol_diff_r3);
    solution3 = traj2fullstates(traj_3);
    solution_set.push_back(solution3); 
    auto traj_4 = generate_traj(file_front + "leader3.yaml");
    traj_set_leader.push_back(traj_4);
	auto traj_fol4 = generate_traj(file_front + "diff_drive3.yaml");
	auto traj_fol_diff_r41 = generate_traj(file_front + "diff_drive13.yaml");
	auto traj_fol_diff_r4 = generate_traj(file_front + "car_like3.yaml");
    traj_set.push_back(traj_4); traj_set.push_back(traj_fol4); traj_set.push_back(traj_fol_diff_r4);
    traj_set.push_back(traj_fol_diff_r41);
    solution4 = traj2fullstates(traj_4);
    solution_set.push_back(solution4);    
    auto traj_5 = generate_traj(file_front + "leader4.yaml");
    traj_set_leader.push_back(traj_5);
	auto traj_fol5 = generate_traj(file_front + "diff_drive4.yaml");
	auto traj_fol_diff_r5 = generate_traj(file_front + "car_like4.yaml");
	auto traj_fol_diff_r51 = generate_traj(file_front + "diff_drive14.yaml");
    traj_set.push_back(traj_5); traj_set.push_back(traj_fol5); traj_set.push_back(traj_fol_diff_r5);
    traj_set.push_back(traj_fol_diff_r51);
    solution5 = traj2fullstates(traj_5);
    solution_set.push_back(solution5);  
    vector<double> robot_type = {1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1};
    std::vector<std::vector<int>> priority_set = generate_priority_set(file_front + "priority.yaml");
    auto plan =  generate_traj_fg("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/ras_demo/formation_generation/fg_16.yaml", 16);
    auto config_ = std::make_shared<PlannerConfig>();
    ros::init (argc, argv, "swarm_traj_planner_rbp");
    ros::NodeHandle nh( "~" );
    ros::Rate rate(10);
    int j = 0;
    SwarmPlanning::Mission mission;
	vector<vector<double>> diff_start, diff_goal;
	vector<vector<double>> car_start, car_goal;
    visualization::Init(nh, "odom", "/hmfpc_test_vis");
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("cisualization_marker", 1);
	math::GenerateObstacle generateobs;
  	std::vector<math::Pose> obstacle;
	std::vector<math::Polygon2d> polys_orig, polys;
	std::vector<std::vector<math::Vec2d>> poly_vertices_set;
    bool plot_traj = false;

    std::vector<ros::Publisher> path_pub_set;
    ros::Publisher path_pub_car1 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_car1", 1, false);
    ros::Publisher path_pub_car2 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_car2", 1, false);
    ros::Publisher path_pub_car3 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_car3", 1, false);
    ros::Publisher path_pub_car4 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_car4", 1, false);
    ros::Publisher path_pub_car5 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_car5", 1, false);
    path_pub_set.push_back(path_pub_car1); path_pub_set.push_back(path_pub_car2); path_pub_set.push_back(path_pub_car3);
    path_pub_set.push_back(path_pub_car4); path_pub_set.push_back(path_pub_car5); 

    // poly_vertices_set.push_back({{-30.5, -30}, {-30.5, 30}, {-30, 30}, {-30, -30}});
    // poly_vertices_set.push_back({{-30, 30}, {-30, 30.5}, {30, 30.5}, {30, 30}});
    // poly_vertices_set.push_back({{30, 30}, {30.5, 30}, {30.5, -30}, {30, -30}});
    // poly_vertices_set.push_back({{-30, -30}, {-30, -30.5}, {30, -30.5}, {30, -30}});

    polys.push_back(math::Polygon2d({{-30.5, -30}, {-30.5, 30}, {-30, 30}, {-30, -30}}));
    polys.push_back(math::Polygon2d({{-30, 30}, {-30, 30.5}, {30, 30.5}, {30, 30}}));
    polys.push_back(math::Polygon2d({{30, 30}, {30.5, 30}, {30.5, -30}, {30, -30}}));
    polys.push_back(math::Polygon2d({{-30, -30}, {-30, -30.5}, {30, -30.5}, {30, -30}}));
    // polys.push_back(math::Polygon2d({{-27, 25}, {-23, 20}, {-17, 21}, {-15, 24}}));
    // polys.push_back(math::Polygon2d({{-20, 14.4}, {-16, 8}, {-6, 9.6}, {-7, 19}}));
    // polys.push_back(math::Polygon2d({{-16, -1}, {-18, -10}, {-6, -9}, {-5.3, -1.0}}));
    // polys.push_back(math::Polygon2d({{5, -8.7}, {2, -17.6}, {12, -24}, {15, -10}}));
    // polys.push_back(math::Polygon2d({{5.6, 15.6}, {3.7, 7.8}, {17, 6.8}, {14.2, 9.6}}));
    // polys.push_back(math::Polygon2d({{16.3, 27.8}, {16.5, 25}, {27.3, 25}, {27.3, 27.9}}));
    // polys.push_back(math::Polygon2d({{-28.4, 3.9}, {-28.3, -6}, {-22.6, -6}, {-20.6, 2.5}}));
    // polys.push_back(math::Polygon2d({{-27.8, -23}, {-26.4, -28.5}, {-16.2, -28.3}, {-17.5, -26.2}}));
    // polys.push_back(math::Polygon2d({{23.2, -16.4}, {15.6, -26}, {23.5, -28}, {28.2, -19}}));
    // polys.push_back(math::Polygon2d({{7.1, -0.3}, {6.8, -2.2}, {19, -2.66}, {20.9, -0.329}}));
    // polys.push_back(math::Polygon2d({{7.1, -0.3}, {6.8, -2.2}, {19, -2.66}, {20.9, -0.329}}));
    // polys.push_back(math::Polygon2d({{-28.9, -14.6}, {-28.6, -20.6}, {-22.3, -22.6}, {-22, -17}}));
    // polys.push_back(math::Polygon2d({{12.7, 22.1}, {7.6, 16.3}, {15.1, 10.4}, {17, 20.3}}));

    // polys.push_back(math::Polygon2d({{-27, 25}, {-23, 20}, {-17, 21}, {-15, 24}}));
    // polys.push_back(math::Polygon2d({{-20, 14.4}, {-16, 9}, {-6, 9.6}, {-14, 16}}));
    // polys.push_back(math::Polygon2d({{-17, -1.5}, {-18, -6}, {-6, -7.6}, {-5.3, -1.9}}));
    // polys.push_back(math::Polygon2d({{5, -8.7}, {3.7, -12.6}, {14, -20}, {15, -10}}));
    // polys.push_back(math::Polygon2d({{5.6, 13.6}, {3.7, 7.8}, {17, 6.8}, {14.2, 9.6}}));
    // polys.push_back(math::Polygon2d({{16.3, 27.8}, {16.5, 25}, {27.3, 25}, {27.3, 27.9}}));
    // polys.push_back(math::Polygon2d({{-28.4, 3.9}, {-28.3, -3.8}, {-22.6, -4}, {-20.6, 0.5}}));
    // // polys.push_back(math::Polygon2d({{-27.8, -23}, {-26.4, -28.5}, {-16.2, -28.3}, {-17.5, -24.7}}));
    // polys.push_back(math::Polygon2d({{-27.8, -23}, {-26.4, -28.5}, {-16.2, -28.3}, {-17.5, -25.8}}));
    // polys.push_back(math::Polygon2d({{23.2, -16.4}, {15.6, -26}, {23.5, -28}, {28.2, -19}}));
    // polys.push_back(math::Polygon2d({{7.1, -1.3}, {6.8, -3.2}, {19, -3.66}, {20.9, -1.329}}));
    polys.push_back(math::Polygon2d({{-27, 25}, {-23, 20}, {-17, 21}, {-15, 24}}));
    polys.push_back(math::Polygon2d({{-20, 14.4}, {-16, 9}, {-6, 9.6}, {-14, 16}}));
    polys.push_back(math::Polygon2d({{-17, 0-8}, {-18, -6-8}, {-6, -7.6-8}, {-5.3, -1.9-8}}));
    polys.push_back(math::Polygon2d({{5, -8.7}, {3.7, -12.6}, {14, -20}, {15, -10}}));
    polys.push_back(math::Polygon2d({{5.6+5, 13.6}, {3.7+5, 7.8}, {17+5, 6.8}, {14.2+5, 9.6}}));
    polys.push_back(math::Polygon2d({{16.3, 27.8}, {16.5, 25}, {27.3, 25}, {27.3, 27.9}}));
    polys.push_back(math::Polygon2d({{-28.4, 3.9}, {-28.3, -3.8}, {-22.6, -4}, {-20.6, 0.5}}));
    polys.push_back(math::Polygon2d({{-27.8, -23}, {-26.4, -28.5}, {-16.2, -28.3}, {-17.5, -24.7}}));
    polys.push_back(math::Polygon2d({{23.2, -16.4}, {15.6, -26}, {23.5, -28}, {28.2, -19}}));
    polys.push_back(math::Polygon2d({{7.1+7, -1.3}, {6.8+7, -3.2}, {19+7, -3.66}, {20.9+7, -1.329}}));
    // obstacle.push_back({0, 0, 0});
    // obstacle.push_back({0, 17, 0});
    // obstacle.push_back({0, -17, 0});

    // obstacle.push_back({-30, 30, 0.1});
	// obstacle.push_back({-15, 30, 0.8});
	// obstacle.push_back({0, 30, 0.3});
	// obstacle.push_back({15, 30, 0.56});
	// obstacle.push_back({30, 30, 0.26});
	// obstacle.push_back({-30, -30, 0.75});
	// obstacle.push_back({-15, -30, 0.83});
	// obstacle.push_back({0, -30, 0.34});
	// obstacle.push_back({15, -30, 0.2});
	// obstacle.push_back({30, -30, 0.98});
	// obstacle.push_back({-30, 15, 0.25});
	// obstacle.push_back({-15, 15, 0.34});
	// obstacle.push_back({0, 15, 0.63});
	// obstacle.push_back({15, 15, 0.45});
	// obstacle.push_back({30, 15, 0.72});
	// obstacle.push_back({-30, -15, 0.23});
	// obstacle.push_back({-12, -15, 0.62});
	// obstacle.push_back({0, -15, 0.27});
	// obstacle.push_back({15, -15, 0.86});
	// obstacle.push_back({30, -15, 0.56});
	// obstacle.push_back({22.5, 0, 0.25});
	// obstacle.push_back({-22.5, 0, 0.89});
    for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
        math::Vec2d centre(obstacle[obs_ind].x(), obstacle[obs_ind].y());
        std::vector<math::Vec2d> poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), false);
        poly_vertices_set.push_back(poly_vertices);
        polys.push_back(math::Polygon2d(poly_vertices));
    } 
    int index_max = 0;
    for (int i = 0; i < traj_set.size(); i++) {
        if (traj_set[i].size() > index_max) {
            index_max = traj_set[i].size();
        }
    }
    if(!mission.setMission(nh)){
        return -1;
    }
    bool ta_finish = false;
    // std::vector<double> theta_set_ = {1.5707963267948966, 1.5707963267948966, 1.5707963267948966, 1.5707963267948966, -0.78539816339744828,
    // -0.78539816339744828, -0.78539816339744828, -0.78539816339744828, -0.78539816339744828, -0.78539816339744828};
    // std::vector<double> theta_set = {-0.78539816339744828, -0.78539816339744828, -0.78539816339744828, -1.5707963267948966, -0.78539816339744828,
    //   -1.5707963267948966, -0.78539816339744828, -0.78539816339744828, -0.78539816339744828, -1.5707963267948966};
    std::vector<double> theta_set_;
    std::vector<double> theta_set;
    for (int i = 0; i < plan.size(); i++) {
        theta_set_.push_back(plan[i].front()[2]);
        // theta_set.push_back(plan[i].back()[2]);
    }
      theta_set.push_back(0.0); theta_set.push_back(0.0); theta_set.push_back(0.78539816339744828); theta_set.push_back(0.78539816339744828); theta_set.push_back(1.5707963267948966);
      theta_set.push_back(0); theta_set.push_back(0.0); theta_set.push_back(0); theta_set.push_back(0.0); theta_set.push_back(0.0);
      theta_set.push_back(0.0); theta_set.push_back(0.78539816339744828); theta_set.push_back(0.78539816339744828); theta_set.push_back(0.0); theta_set.push_back(0.0);
      theta_set.push_back(0.0); theta_set.push_back(1.5707963267948966); theta_set.push_back(1.5707963267948966);
    // while (ros::ok()) {
    //     // for (int i = 0; i < 4; i++) {
    //     //     visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Grey, i, "Boundary"+  std::to_string(i));
    //     // }
    //     for (int i = 4; i < polys.size(); i++) {
    //         visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Red, i, "Obstacle"+  std::to_string(i));
    //     }
    //     for (int i = 0; i < mission.qn; i++) {
    //         math::Pose start(mission.startState[i][0], mission.startState[i][1], theta_set_[i]);
    //         math::Pose goal(mission.goalState[i][0], mission.goalState[i][1], theta_set[i]);
    //         std::vector<double> xs = {mission.startState[i][0], mission.goalState[i][0]};
    //         std::vector<double> ys = {mission.startState[i][1], mission.goalState[i][1]};
    //         auto box_s = config_->vehicle.GenerateBox(start);
    //         auto box_g = config_->vehicle.GenerateBox(goal);
    //         auto color = mission.quad_type[i] > 0 ? visualization::Color::Green : visualization::Color::Cyan;
    //         const std::string robot_name_ta = "TaskAssignment" + std::to_string(i);
    //         const std::string robot_name_s = "Footprint_s_" + std::to_string(i);
    //         const std::string robot_name_g = "Footprint_g_" + std::to_string(i);
    //         visualization::PlotPolygon(math::Polygon2d(box_s), 0.3, color, i, robot_name_s);
    //         color.set_alpha(0.4);
    //         visualization::PlotPolygon(math::Polygon2d(box_g), 0.3, color, i, robot_name_g);
    //         // color = mission.quad_type[i] > 0 ? visualization::Color::Green : visualization::Color::Cyan;
    //         color = visualization::Color::Yellow;
    //         color.set_alpha(0.4);
    //         visualization::Plot(xs, ys, 0.2, color, i, robot_name_ta);
    //     }
  	// 	visualization::Trigger();
    //     j++;
    //     if (j == 100) {
    //         for (int i = 0; i < mission.qn; i++) {
    //             visualization::Delete(i, "TaskAssignment" + std::to_string(i));
    //             visualization::Delete(i, "Footprint_s_" + std::to_string(i));
    //             visualization::Delete(i, "Footprint_g_" + std::to_string(i));
    //         }
    //         visualization::Trigger();
    //         break;
    //     }
    //     ros::spinOnce();
    //     rate.sleep();	
    // }
    j = 0;
    while (ros::ok()) {
        // PlotTrajectoryFG(config_, plan);
        // for (int i = 0; i < 4; i++) {
        // visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Grey, i, "Boundary"+  std::to_string(i));
        // }
        for (int i = 4; i < polys.size(); i++) {
        visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Red, i, "Obstacle"+  std::to_string(i));
        }
		for (int i = 0; i < plan.size(); i++) {
            PlotPoseFG(config_, i, j, plan, mission.quad_type);
		}
  		visualization::Trigger();
        j++;
        if (j == plan[0].size()) {
            for (int i = 0; i < plan.size(); i++) {
                for (int j_ = 0; j_ < plan[i].size(); j_++) {
                    visualization::Delete(j_, "FGTraj_" + std::to_string(i));
                }
            }
            visualization::Trigger();           
            break;
        }
        ros::spinOnce();
        rate.sleep();	
	}
    // j =0;
    // while (ros::ok()) {
    //     // if (!plot_traj) {    
    //         for (int traj_ind = 0; traj_ind < solution_set.size(); traj_ind++) {
    //             DrawTrajectoryRviz(solution_set[traj_ind], config_, traj_ind, path_pub_set[traj_ind]);
    //         }
    //         // plot_traj = true;
    //     // }
   	// 	for (int i = 0; i < priority_set.size(); i++) {
    //         for(int j = 0; j < 1e3; j++) {
    //             visualization::Delete(j, "RobotArrow" + std::to_string(i));
    //         }
    //     }
    //     visualization::Trigger();
    //     // visualization_msgs::Marker arrow;
    //     // arrow.header.frame_id = "map";
    //     // arrow.header.stamp = ros::Time::now();
    //     // arrow.ns = "arrow_namespace";
    //     // arrow.id = 0;
    //     // arrow.type = visualization_msgs::Marker::ARROW;
    //     // arrow.action = visualization_msgs::Marker::ADD;
    //     // arrow.pose.position.x = 1.0;
    //     // arrow.pose.position.y = 1.0;
    //     // arrow.pose.position.z = 0.0;

    //     // // 计算箭头方向，角度为45度
    //     // double angle = 45.0 * M_PI / 180.0;
    //     // arrow.pose.orientation.w = cos(angle / 2);
    //     // arrow.pose.orientation.z = sin(angle / 2);

    //     // arrow.scale.x = 10.0;  // Increase the size
    //     // arrow.scale.y = 0.5;
    //     // arrow.scale.z = 0.5;
    //     // arrow.color.a = 1.0;
    //     // arrow.color.r = 1.0;
    //     // arrow.color.g = 0.0;
    //     // arrow.color.b = 0.0;

    //     // // 发布Marker消息
    //     // marker_pub.publish(arrow);
    //     // for (int i = 0; i < 4; i++) {
    //     //     visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Grey, i, "Boundary"+  std::to_string(i));
    //     // }
    //     for (int i = 4; i < polys.size(); i++) {
    //         visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Red, i, "Obstacle"+  std::to_string(i));
    //     }
	// 	for (int i = 0; i < traj_set.size(); i++) {
    //         PlotPose(config_, i, j, traj_set[i], robot_type);
	// 	}
  	// 	visualization::Trigger();
   	// 	for (int i = 0; i < priority_set.size(); i++) {
    //         if (priority_set[i][j] != -1) {
    //             auto color = visualization::Color::Magenta;
	// 		    // color.set_alpha(0.4);
    //             if (j <= traj_set_leader[i].size() - 1) {
    //                 std::vector<double> xs = {traj_set_leader[i][j].x, 
    //                     traj_set_leader[priority_set[i][j]][std::min(int(traj_set_leader[priority_set[i][j]].size()-1), j)].x};
    //                 std::vector<double> ys = {traj_set_leader[i][j].y, 
    //                     traj_set_leader[priority_set[i][j]][std::min(int(traj_set_leader[priority_set[i][j]].size()-1), j)].y};
    //                 const std::string robot_name_ta = "RobotArrow" + std::to_string(i);
    //                 visualization::Plot(xs, ys, 0.5, color, j, robot_name_ta);
    //                 visualization::Trigger();  
    //            }
    //             else {
    //                 std::vector<double> xs = {traj_set_leader[i][traj_set_leader[i].size()-1].x, 
    //                     traj_set_leader[priority_set[i][j]][std::min(int(traj_set_leader[priority_set[i][j]].size()-1), j)].x};
    //                 std::vector<double> ys = {traj_set[i*3][traj_set[i*3].size()-1].y, 
    //                     traj_set_leader[priority_set[i][j]][std::min(int(traj_set_leader[priority_set[i][j]].size()-1), j)].y};
    //                 const std::string robot_name_ta = "RobotArrow" + std::to_string(i);
    //                 visualization::Plot(xs, ys, 0.5, color, j, robot_name_ta);
    //                 visualization::Trigger();                 
    //             }        
    //         } 
	// 	} 
    //     j++;
    //     if (j == index_max) {
    //         break;
    //     }
    //     ros::spinOnce();
    //     rate.sleep();	
	// }

	return 0;
}

// ps aux | grep ros |  awk '{print $2}' | xargs kill -9; ps aux | grep rviz |  awk '{print $2}' | xargs kill -9
/**
 * file animation.cpp
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief animation
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
#include "traj_tracking/matplotlibcpp.h"
#include <nav_msgs/Path.h>
#include "heterogeneous_formation_controller/forward_kinematics.h"

using namespace heterogeneous_formation_controller;
using namespace forward_kinematics;
namespace plt = matplotlibcpp;

void generateRegularPolygon(const double r, const int k, 
  std::vector<std::vector<double>>& vertice_set) {
    double cx = 0.0, cy = 0.0;
    std::vector<std::pair<double, double>> vertices;
    double angleIncrement = 2 * M_PI / k;
    for (int i = 0; i < k; ++i) {
        TrajectoryPoint tp_temp;
        double angle = i * angleIncrement;
        double x = cx + r * std::cos(angle);
        double y = cy + r * std::sin(angle);
        tp_temp.x = x;
        tp_temp.y = y;
        tp_temp.theta = 0.0;
        vertice_set.push_back({x, y});
    }
}


void DrawTrajectoryRviz(const FullStates sol_traj, std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config,
      int robot_index, ros::Publisher path_pub, double inflat) {
  for (int i = 0; i < sol_traj.states.size(); i++) {
    auto x0_disc = config->vehicle.GetVertexPositions(sol_traj.states[i].x, sol_traj.states[i].y, sol_traj.states[i].theta, inflat);
    std::vector<double> x1, y1;
    x1.push_back(x0_disc[0]);x1.push_back(x0_disc[2]);x1.push_back(x0_disc[4]);x1.push_back(x0_disc[6]);x1.push_back(x0_disc[0]);
    y1.push_back(x0_disc[1]);y1.push_back(x0_disc[3]);y1.push_back(x0_disc[5]);y1.push_back(x0_disc[7]);y1.push_back(x0_disc[1]);
    auto color = visualization::Color::White;
    color.set_alpha(0.1);
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

void PlotPose(std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config, int robot_index, int time_step, Trajectory_temp traj, std::vector<double> robot_type, const double& xo, const double& yo) {
    std::vector<double> xo_set, yo_set;
    std::vector<double> xo_, yo_;
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
    xo_set.push_back(robot_pos.x());
    xo_set.push_back(xo);
    xo_set.push_back(robot_pos.x());
    yo_set.push_back(robot_pos.y());
    yo_set.push_back(yo);
    yo_set.push_back(robot_pos.y());
    visualization::Plot(xo_set,yo_set, 0.5, visualization::Color::Green, 1, "cable" + std::to_string(robot_index));
    auto box = config->vehicle.GenerateBox(robot_pos);
    auto color = robot_type[robot_index] > 0 ? visualization::Color::Red : visualization::Color::Cyan;
    // color.set_alpha(0.4);
    xo_.push_back(xo+0.3); xo_.push_back(xo+0.3); xo_.push_back(xo-0.3); xo_.push_back(xo-0.3); xo_.push_back(xo+0.3);
    yo_.push_back(yo+0.3); yo_.push_back(yo-0.3); yo_.push_back(yo-0.3); yo_.push_back(yo+0.3); yo_.push_back(yo+0.3);
    visualization::Plot(xo_, yo_, 0.5, visualization::Color::Grey, 1, "point_s");
    visualization::Trigger();
    visualization::PlotPolygon(math::Polygon2d(box), 0.5, color, 1, robot_name);
    visualization::Trigger();
    // visualization::Plot(xs, ys, 0.5, visualization::Color::Magenta, i, robot_name_ta);
}
int main(int argc, char* argv[])
{
    VVCM vvcm;
    std::vector<std::vector<double>> pos_3d;
    std::vector<std::vector<double>> pos_2d; 
    std::vector<std::vector<int>> taut_set;
    std::vector<std::vector<double>> vertice_set;
    std::vector<std::vector<double>> robot_pos_set;
    generateRegularPolygon(4.05 / sqrt(3), num_robot, vertice_set);
    ForwardKinematics fk_test(num_robot, vvcm.zr, vertice_set, robot_pos_set);
	std::vector<Trajectory_temp> traj_set;
    std::vector<FullStates> solution_set;
    FullStates solution1, solution2, solution3, solution4, solution5, solution6;
    std::string file_front = "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/flexible_formation/";
    std::string file_back = ".yaml";
    int traj_ind = 1;
    for (int i = 0; i < num_robot; i++) {
        auto traj_temp = generate_traj(file_front + "traj_" + std::to_string(num_robot)+ std::to_string(i) + ".yaml");
        traj_set.push_back(traj_temp); 
    }
    vector<double> robot_type = {1, 1, 1, 1, 1, 1, 1, 1, 1};
    // std::vector<std::vector<int>> priority_set = generate_priority_set(file_front + "priority.yaml");
    auto config_ = std::make_shared<PlannerConfig>();
    ros::init (argc, argv, "swarm_traj_planner_rbp");
    ros::NodeHandle nh( "~" );
    ros::Rate rate(50);
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
    path_pub_set.push_back(path_pub_car1); path_pub_set.push_back(path_pub_car2); path_pub_set.push_back(path_pub_car3);

    // polys.push_back(math::Polygon2d({{-30.5, -30}, {-30.5, 30}, {-30, 30}, {-30, -30}}));
    // polys.push_back(math::Polygon2d({{-30, 30}, {-30, 30.5}, {30, 30.5}, {30, 30}}));
    // polys.push_back(math::Polygon2d({{30, 30}, {30.5, 30}, {30.5, -30}, {30, -30}}));
    // polys.push_back(math::Polygon2d({{-30, -30}, {-30, -30.5}, {30, -30.5}, {30, -30}}));
    // polys.push_back(math::Polygon2d({{-10, -0.5}, {-10, 0.5}, {10, 0.5}, {10, -0.5}}));
    // polys.push_back(math::Polygon2d({{-4, 5}, {4, 5}, {4, 3}, {-4, 3}}));
    // polys.push_back(math::Polygon2d({{-4, -5}, {4, -5}, {4, -2.5}, {-4, -2.5}}));

    // obstacle.push_back({0, 11.5, 0});
    // obstacle.push_back({0, -11.5, 0});
    obstacle.push_back({0, 9.5, 0});
    obstacle.push_back({0, -9.5, 0});
	// obstacle.push_back({18.5, 0, 0.25});
	// obstacle.push_back({-18.5, 0, 0.3});    	
    obstacle.push_back({18.5, 0, 0.25});
	obstacle.push_back({-18.5, 0, 0.3});
	// obstacle.push_back({0, 0, 0});
    std::vector<double> x_, y_, y__;
    std::vector<double> xo, yo;
    for (int i = 0; i < traj_set[0].size(); i++) {
        robot_pos_set.clear();
        pos_2d.clear();
        pos_3d.clear();
        taut_set.clear();
        double po_x = 0.0, po_y = 0.0, po_z = -0.0;
        for (int j = 0; j < traj_set.size(); j++) {
            robot_pos_set.push_back({traj_set[j][i].x, traj_set[j][i].y});
        }
        fk_test.SolveFk(vertice_set, robot_pos_set, pos_3d, pos_2d, taut_set, vvcm.zr);
        for (int po_count = 0; po_count < pos_3d.size(); po_count++) {
            po_x += pos_3d[po_count][0];
            po_y += pos_3d[po_count][1];
            po_z += pos_3d[po_count][2];
        }
        xo.push_back(po_x / pos_3d.size());
        yo.push_back(po_y / pos_3d.size());
        y_.push_back(po_z / pos_3d.size());
        x_.push_back(traj_set[0][i].t);
    }

    for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
        math::Vec2d centre(obstacle[obs_ind].x(), obstacle[obs_ind].y());
        std::vector<math::Vec2d> poly_vertices;
        if (obs_ind < 2) {    
            poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), true);
        }
        else {
            poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), false);
        }

        poly_vertices_set.push_back(poly_vertices);
        polys.push_back(math::Polygon2d(poly_vertices));
    } 
    int index_max = traj_set[0].size();
    auto color = visualization::Color::Blue;
    while (ros::ok()) {
        // visualization::Trigger();
        // if (!plot_traj) {    
        // for (int traj_ind = 0; traj_ind < solution_set.size(); traj_ind++) {
        //     DrawTrajectoryRviz(solution_set[traj_ind], config_, traj_ind, path_pub_set[traj_ind], inflat_set[traj_ind]);

        // }
        visualization::Trigger();
        for (int i = 0; i < 4; i++) {
            visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Black, i, "Boundary"+  std::to_string(i));
        }
        for (int i = 4; i < polys.size(); i++) {
            visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Black, i, "Obstacle"+  std::to_string(i));
        }
		for (int i = 0; i < traj_set.size(); i++) {
            PlotPose(config_, i, j, traj_set[i], robot_type, xo[j], yo[j]);
		}
  		visualization::Trigger();
        j++;
        if (j == index_max) {
            break;
        }
        ros::spinOnce();
        rate.sleep();	
	}
    writeVectorToYAML(y_, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/flexible_formation/height.yaml");
    plt::clf();
    plt::xlabel("time step");
    plt::ylabel("height of the object/m");
    plt::plot(x_, y_);
    // plt::plot(obs1_x, obs1_y);
    // plt::plot(x_, y__);
    plt::show();
	return 0;
}

// ps aux | grep ros |  awk '{print $2}' | xargs kill -9; ps aux | grep rviz |  awk '{print $2}' | xargs kill -9
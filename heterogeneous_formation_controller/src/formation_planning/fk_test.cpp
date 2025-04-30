/**
 * file fk_test.cpp
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief animation
 * data 2024-06-14
 * 
 * @copyright Copyroght(c) 2024
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
#include "heterogeneous_formation_controller/forward_kinematics.h"
#include <nav_msgs/Path.h>

using namespace heterogeneous_formation_controller;
using namespace forward_kinematics;
namespace plt = matplotlibcpp;

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
    ros::init (argc, argv, "swarm_traj_planner_rbp");
    ros::NodeHandle nh( "~" );
    ros::Rate rate(50);
    visualization::Init(nh, "odom", "/hmfpc_test_vis");
    int num_robot = 4;
    double height = 0.8;
    std::vector<std::vector<double>> vertice_initial_set = {
        {-0.32, -0.42},
        {0.80, -0.38},
        {0.75, 0.71}, 
        {-0.37, 0.66}
    };
    std::vector<std::vector<double>> robot_pos_set = {
        {0.21, 0.12},
        {0.80, 0.04},
        {0.90, 0.55},
        {0.44, 0.72}
    };
    std::vector<std::vector<double>> pos_3d;
    std::vector<std::vector<double>> pos_2d; 
    std::vector<std::vector<int>> taut_set;
    ForwardKinematics fk_test(num_robot, height, vertice_initial_set, robot_pos_set);
    while (ros::ok()) {
        fk_test.SolveFk(vertice_initial_set, robot_pos_set, pos_3d, pos_2d, taut_set, height);
        break;
        ros::spinOnce();
        rate.sleep();	
	}
    // writeVectorToYAML(y_, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/flexible_formation/height.yaml");
    plt::clf();
    plt::xlabel("x/m");
    plt::ylabel("y/m");
    int index = 1;
    std::vector<double> xo = {pos_3d[index][0], pos_3d[index][0]};
    std::vector<double> yo = {pos_3d[index][1], pos_3d[index][1]};
    plt::scatter(xo, yo, 200.0, {{"color", "red"}, {"marker", "o"}}); 
    std::vector<double> obs_x, obs_y;
    for (int i = 0; i <= robot_pos_set.size(); i++) {
        obs_x.push_back(robot_pos_set[i % robot_pos_set.size()][0]);
        obs_y.push_back(robot_pos_set[i % robot_pos_set.size()][1]);
    }
    plt::plot({pos_3d[index][0], robot_pos_set[0][0]}, {pos_3d[index][1], robot_pos_set[0][1]}, "b-");
    plt::plot({pos_3d[index][0], robot_pos_set[1][0]}, {pos_3d[index][1], robot_pos_set[1][1]}, "b--"); 
    plt::plot({pos_3d[index][0], robot_pos_set[2][0]}, {pos_3d[index][1], robot_pos_set[2][1]}, "b-"); 
    plt::plot({pos_3d[index][0], robot_pos_set[3][0]}, {pos_3d[index][1], robot_pos_set[3][1]}, "b-"); 
    plt::plot(xo, yo, "r-"); 
    plt::plot(obs_x, obs_y, "k-"); 
    plt::show();
	return 0;
}

// ps aux | grep ros |  awk '{print $2}' | xargs kill -9; ps aux | grep rviz |  awk '{print $2}' | xargs kill -9
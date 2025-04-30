/**
 * file fg_test_node.cpp
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief formation generation test 
 * data 2023-11-22
 * 
 * @copyright Copyroght(c) 2023
*/
#include <ros/ros.h>
#include <memory>
#include <utility>
#include <cmath>
#include "heterogeneous_formation_controller/heterogeneous_formation_controller.h"
#include "heterogeneous_formation_controller/visualization/plot.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>

#include "iris/iris.h"
#include <Eigen/Core>
#include "heterogeneous_formation_controller/yaml_all.h"
#include "heterogeneous_formation_controller/math/generate_obs.h"
#include <Eigen/Core>
#include <math.h>
#include <environment.hpp>

// Parameters
#include <formation_generation/param.hpp>
#include <formation_generation/mission.hpp>
#include <formation_generation/timer.hpp>

// Submodules
#include <formation_generation/ecbs_planner.hpp>
#include <formation_generation/corridor.hpp>
#include <formation_generation/prioritized_traj_optimization.hpp>
#include <formation_generation/prioritized_traj_publisher.hpp>

#include <decomp_util/decomp_util/seed_decomp.h>
#include <decomp_util/decomp_geometry/geometric_utils.h>
#include <fstream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

using namespace heterogeneous_formation_controller;
bool has_octomap = false;
bool has_path = false;
vector<heterogeneous_formation_controller::visualization::Color> colors = {
  visualization::Color::Red,
  visualization::Color::Red,
  visualization::Color::Blue,
  visualization::Color::Blue,
  // visualization::Color::Grey,
  visualization::Color::Magenta,
  visualization::Color::Magenta,
  visualization::Color::Yellow,
  visualization::Color::Yellow,
  
  visualization::Color::Magenta,
  visualization::Color::Magenta,
  visualization::Color::Yellow,
  visualization::Color::Yellow,
  // visualization::Color::Grey,
  visualization::Color::Red,
  visualization::Color::Red,
  visualization::Color::Blue,
  visualization::Color::Blue,
  // visualization::Color::Grey,
  visualization::Color::Yellow,
  visualization::Color::Magenta,
  // visualization::Color::Grey,
  visualization::Color::Magenta,
  visualization::Color::Blue,
  // visualization::Color::White,
  visualization::Color::Blue,
  visualization::Color::Red,
  // visualization::Color::Black,
  visualization::Color::Green,
  visualization::Color::Green,
  visualization::Color::Cyan
};
std::shared_ptr<octomap::OcTree> octree_obj;

std::vector<double> CalAvgFlowtime(const std::vector<std::vector<std::vector<double>>>& plan) {
  std::vector<double> comp_time;
  std::vector<double> time_set;
  std::vector<double> distance_set;
  std::vector<int> terminal_time;
  double makespan = 0.0;
  int comp_ind = 0;
  for (int i = 0; i < plan.size(); i++) {
    for (int j = plan[i].size() - 1; j > 0; j--) {
      if (plan[i][j][0] != 0 && plan[i][j][1] != 0) {
        break;
      }
      terminal_time.push_back(j);
    }
  }
  for (int i = 0; i < plan.size(); i++) {
    for (int j = terminal_time[i]; j > 0; j--) {
      if (pow(plan[i][j][0] - plan[i][plan.size() - 1][0], plan[i][j][1] - plan[i][plan.size() - 1][1]) > 0.3) {
        comp_ind = j;
        break;
      }
    }
    comp_time.push_back(comp_ind * 0.32);
  }
  double distance_avg = 0.0;
  for (int i = 0; i < plan.size(); i++) {
    double distance = 0.0;
    for (int j = terminal_time[i]; j > 1; j--) {
      distance += hypot(plan[i][j][0]-plan[i][j-1][0],plan[i][j][1]-plan[i][j-1][1]);
    }
    distance_set.push_back(distance);
  }
  for (int i = 0; i < distance_set.size(); i++) {
    distance_avg += distance_set[i];
  }
  distance_avg /= 15;
  double time_all = 0.0;
  for (int i = 0; i < comp_time.size(); i++) {
    time_all += comp_time[i];
    if (makespan < comp_time[i]) {
      makespan = comp_time[i];
    }
  }
  time_set.push_back(time_all / comp_time.size());
  time_set.push_back(makespan);
  return time_set;
}

void DrawTrajectoryRviz(const FullStates sol_traj, std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config,
      int robot_index, ros::Publisher path_pub) {
  const std::string robot_name = "Footprint_" + std::to_string(robot_index);
  for(int i = 0; i < 1e3; i++) {
    visualization::Delete(i, robot_name);
  }
  visualization::Trigger();

  nav_msgs::Path msg;
  msg.header.frame_id = "odom";
  msg.header.stamp = ros::Time::now();
  for(size_t i = 0; i < sol_traj.states.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = sol_traj.states[i].x;
    pose.pose.position.y = sol_traj.states[i].y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(sol_traj.states[i].theta);
    msg.poses.push_back(pose);

    auto box = config->vehicle.GenerateBox(sol_traj.states[i].pose());
    auto color = robot_index > 0 ? visualization::Color::Yellow : visualization::Color::White;
    color.set_alpha(0.4);
    visualization::PlotPolygon(math::Polygon2d(box), 0.05, color, i, robot_name);
  }
  path_pub.publish(msg);
  visualization::Trigger();
}

void DrawCorasePathRivz(const std::vector<octomap::point3d> path, int robot_index) {
  std::vector<double> xs, ys;
  const std::string robot_name = "Footprint_" + std::to_string(robot_index);
  for (int i = 0; i < path.size(); i++) {
    xs.push_back(path[i].x());
    ys.push_back(path[i].y());
  }
  visualization::Plot(xs, ys, 1, colors[robot_index], robot_index, robot_name);
  visualization::Trigger();
}

void DrawFGTrajRivz(const std::vector<vector<double>> traj, int robot_index, int traj_length) {
  std::vector<double> xs, ys;
  const std::string robot_name = "Footprint_" + std::to_string(robot_index);
  for (int i = 0; i < traj_length; i++) {
    xs.push_back(traj[i][0]);
    ys.push_back(traj[i][1]);
  }
  visualization::Plot(xs, ys, 0.5, colors[robot_index], robot_index, robot_name);
  visualization::Trigger();
}

std::vector<std::vector<std::vector<double>>> InsertPath(int qn, int M, std::vector<std::vector<octomap::point3d>> initTraj) {
  int i, j, k;
  int N = M * 5 + 1;
  double dx;
  double dy;
  double dis;
  std::vector<std::vector<std::vector<double>>> threeDArray(qn, std::vector<std::vector<double>>(M, std::vector<double>(3)));

  for (i = 0; i < qn; i++){
    for(j = 0; j < M; j++){

        dx = (initTraj[i][j+1].x() - initTraj[i][j].x()) / 5;
        dy = (initTraj[i][j+1].y() - initTraj[i][j].y()) / 5;
        dis = 25 * (dx * dx + dy * dy); 
        for (k = 0; k < 5; k++){
            plan[i][5*j+k][0] = initTraj[i][j].x() + dx*k;
            plan[i][5*j+k][1] = initTraj[i][j].y() + dy*k;

            if (dis < 0.1)
            {
              plan[i][5*j+k][3] = 0;
            }
            if (dis < 1.1){
              plan[i][5*j+k][3]=MAXV*0.7;
            }
            else{
              plan[i][5*j+k][3]=MAXV;
            }
        }
    }
    plan[i][5*M][0] = initTraj[i][M].x();
    plan[i][5*M][1] = initTraj[i][M].y();
  }

  N = M*5+1;
  // 配准朝向角
  for (int qi = 0; qi < qn; qi++)
  {
    for (int next = 5; next < N; next += 5){
      if ((plan[qi][next][1] == plan[qi][0][1]) && (plan[qi][next][0] == plan[qi][0][0]))
        continue;
      else{
        plan[qi][0][2] = atan2(plan[qi][next][1] - plan[qi][0][1],plan[qi][next][0] - plan[qi][0][0]);
        break;
      }
    }
  }
  double angle;
  for (i = 0; i < qn; i++){
    for(j = 0; j < M; j++){

      dx = initTraj[i][j+1].x() - initTraj[i][j].x();
      dy = initTraj[i][j+1].y() - initTraj[i][j].y();
      dis =  dx * dx + dy * dy;
      
      if (j > 0){
        if (dis > 0.1){
          angle = atan2(dy, dx);
          if (angle - plan[i][5*(j-1)][2] > PI)
            angle = angle - 2*PI;
          else if(plan[i][5*(j-1)][2] - angle > PI)
            angle = angle + 2*PI;
        }
          else angle = plan[i][5*(j-1)][2];
      }
      else {angle = plan[i][0][2];}

      for (k = 0; k < 5; k++){
        plan[i][5*j+k][2] = angle;
      } 
    }
  }
  return plan;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "hmfpc_test_node");
  // 它创建的节点句柄将使用波浪线作为命名空间，这意味着它将访问和处理与当前节点相关的参数和话题的私有命名空间。
  // 这对于在ROS中编写节点时非常有用，因为它允许您将参数和话题隔离在节点的私有命名空间中，以避免与其他节点的
  // 参数和话题发生冲突。
  ros::NodeHandle nh( "~" );
  auto config_ = std::make_shared<PlannerConfig>();
  config_->vehicle.InitializeDiscs();

  auto env = std::make_shared<Environment>(config_);
  auto planner_ = std::make_shared<hmfpcLocalPlanner>(config_, env);

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/hmfpc_test_path", 1, false);
  ros::Publisher path_pub_diff1 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_diff1", 1, false);
  ros::Publisher path_pub_diff2 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_diff2", 1, false);
  ros::Publisher path_pub_car1 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_car1", 1, false);

  interactive_markers::InteractiveMarkerServer server_("/hmfpc_obstacle");
  // vec_Vec2f obs;
  math::GenerateObstacle generateobs;
  std::vector<math::Pose> obstacle;
  std::vector<math::Polygon2d> polys_inflat, polys, polys_inflat_ecbs;
  // std::vector<std::vector<math::Vec2d>> poly_vertices_set, poly_vertices_set_inflated;

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
	// // obstacle.push_back({-15, -15, 0.62});
	// obstacle.push_back({0, -15, 0.27});
	// obstacle.push_back({15, -15, 0.86});
	// obstacle.push_back({30, -15, 0.56});
	// obstacle.push_back({22.5, 0, 0.25});
	// obstacle.push_back({-22.5, 0, 0.89});
    // polys.push_back(math::Polygon2d({{-27, 25}, {-23, 20}, {-17, 21}, {-15, 24}}));
    // polys.push_back(math::Polygon2d({{-20, 14.4}, {-16, 9}, {-6, 9.6}, {-14, 16}}));
    // polys.push_back(math::Polygon2d({{-17, 0}, {-18, -6}, {-6, -7.6}, {-5.3, -1.9}}));
    // polys.push_back(math::Polygon2d({{5, -8.7}, {3.7, -12.6}, {14, -20}, {15, -10}}));
    // polys.push_back(math::Polygon2d({{5.6, 13.6}, {3.7, 7.8}, {17, 6.8}, {14.2, 9.6}}));
    // polys.push_back(math::Polygon2d({{16.3, 27.8}, {16.5, 25}, {27.3, 25}, {27.3, 27.9}}));
    // polys.push_back(math::Polygon2d({{-28.4, 3.9}, {-28.3, -3.8}, {-22.6, -4}, {-20.6, 0.5}}));
    // polys.push_back(math::Polygon2d({{-27.8, -23}, {-26.4, -28.5}, {-16.2, -28.3}, {-17.5, -24.5}}));
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
  // polys.push_back(math::Polygon2d({{-33 + 0.3, 28.8 + 0.3}, {-33 - 0.3, 28.8 + 0.3}, {-33 - 0.3, 28.8 - 0.3}, {-33 + 0.3, 28.8 - 0.3}}));
  // polys.push_back(math::Polygon2d({{-33 + 0.3, 21 + 0.3}, {-33 - 0.3, 21 + 0.3}, {-33 - 0.3, 21 - 0.3}, {-33 + 0.3, 21 - 0.3}}));
  // polys.push_back(math::Polygon2d({{-20 + 0.3, 28.8 + 0.3}, {-20 - 0.3, 28.8 + 0.3}, {-20 - 0.3, 28.8 - 0.3}, {-20 + 0.3, 28.8 - 0.3}}));
  // polys.push_back(math::Polygon2d({{-20 + 0.3, 21 + 0.3}, {-20 - 0.3, 21 + 0.3}, {-20 - 0.3, 21 - 0.3}, {-20 + 0.3, 21 - 0.3}}));
  // polys.push_back(math::Polygon2d({{ -43.7, 52.9}, {-74.3, 52.7}, {-73.7, 45.2}, {-45.2, 45.5}}));
  // polys.push_back(math::Polygon2d({{40 + 0.5, 34 + 0.5}, {40 - 0.5, 34 + 0.5}, {40 - 0.5, 34 - 0.5}, {40 + 0.5, 34 - 0.5}}));
  // polys.push_back(math::Polygon2d({{58 + 0.5, 33.6 + 0.5}, {58 - 0.5, 33.6 + 0.5}, {58 - 0.5, 33.6 - 0.5}, {58+ 0.5, 33.6 - 0.5}}));
  // polys.push_back(math::Polygon2d({{40 + 0.5, 20.6 + 0.5}, {40 - 0.5, 20.6 + 0.5}, {40 - 0.5, 20.6 - 0.5}, {40 + 0.5, 20.6 - 0.5}}));
  // polys.push_back(math::Polygon2d({{58 + 0.5, 20.5 + 0.5}, {58 - 0.5, 20.5 + 0.5}, {58 - 0.5, 20.6 - 0.5}, {58 + 0.5, 20.6 - 0.5}}));  
  // // polys.push_back(math::Polygon2d({{ -74, 52.2}, {-81.2, 52.5}, {-74.3, 10}, {-81.2, 10}})); 
  // polys.push_back(math::Polygon2d({{ -74, 10}, {-73.6, 16.1}, {-51.3, 22.1}, {-51, 10.7}})); 
  // polys.push_back(math::Polygon2d({{ -49.7, 10}, {-53.1, 10}, {-52.3, 0}, {-51, 0}})); 
  // polys.push_back(math::Polygon2d({{ -49.7, 0}, {-49.1,  -2}, {75, -2}, {75, 0}})); 
  // polys.push_back(math::Polygon2d({{ -42.6, 51.3}, {-1.3,  51.1}, {-1.6, 56}, {-46.1,  56}})); 
  // polys.push_back(math::Polygon2d({{ -0.65, 52.5}, {-1.3,  7}, {24.2, 7}, {23.6,  51.2}})); 
  // polys.push_back(math::Polygon2d({{ 39.5 + 1, 20 + 1}, {39.5 - 1,  20 + 1}, {39.5 - 1, 20 - 1}, {39.5 + 1,  20 - 1}})); 
  // polys.push_back(math::Polygon2d({{ 72.5, -1.5}, {76.7,  -1.5}, {76.7, 50}, {72.5,  50}})); 
	// polys.push_back(math::Polygon2d({{-30.5, -30}, {-30.5, 30}, {-30, 30}, {-30, -30}}));
	// polys.push_back(math::Polygon2d({{-30, 30}, {-30, 30.5}, {30, 30.5}, {30, 30}}));
	// polys.push_back(math::Polygon2d({{30, 30}, {30.5, 30}, {30.5, -30}, {30, -30}}));
	// polys.push_back(math::Polygon2d({{-30, -30}, {-30, -30.5}, {30, -30.5}, {30, -30}}));
	// obstacle.push_back({0, 0, 0});
	// obstacle.push_back({0, 17, 0});
	// obstacle.push_back({0, -17, 0});
  // polys.push_back(math::Polygon2d({{-30.5, -30}, {-30.5, 30}, {-30, 30}, {-30, -30}}));
  // polys.push_back(math::Polygon2d({{-30, 30}, {-30, 30.5}, {30, 30.5}, {30, 30}}));
  // polys.push_back(math::Polygon2d({{30, 30}, {30.5, 30}, {30.5, -30}, {30, -30}}));
  // polys.push_back(math::Polygon2d({{-30, -30}, {-30, -30.5}, {30, -30.5}, {30, -30}}));
  // polys.push_back(math::Polygon2d({{-27, 25}, {-23, 20}, {-17, 21}, {-15, 24}}));
  // polys.push_back(math::Polygon2d({{-20, 14.4}, {-16, 8}, {-6, 9.6}, {-7, 19}}));
  // polys.push_back(math::Polygon2d({{-16, -1}, {-18, -10}, {-6, -9}, {-5.3, -1}}));
  // polys.push_back(math::Polygon2d({{5, -8.7}, {2, -17.6}, {12, -24}, {15, -10}}));
  // polys.push_back(math::Polygon2d({{5.6, 15.6}, {3.7, 7.8}, {17, 6.8}, {14.2, 9.6}}));
  // polys.push_back(math::Polygon2d({{16.3, 27.8}, {16.5, 25}, {27.3, 25}, {27.3, 27.9}}));
  // polys.push_back(math::Polygon2d({{-28.4, 3.9}, {-28.3, -6}, {-22.6, -6}, {-20.6, 2.5}}));
  // polys.push_back(math::Polygon2d({{-27.8, -23}, {-26.4, -28.5}, {-16.2, -28.3}, {-17.5, -25.2}}));
  // polys.push_back(math::Polygon2d({{23.2, -16.4}, {15.6, -26}, {23.5, -28}, {28.2, -19}}));
  // polys.push_back(math::Polygon2d({{7.1, -0.3}, {6.8, -2.2}, {19, -2.66}, {20.9, -0.329}}));
  
  // polys.push_back(math::Polygon2d({{-27, 25}, {-23, 20}, {-17, 21}, {-15, 24}}));
  // polys.push_back(math::Polygon2d({{-20, 14.4}, {-16, 9}, {-6, 9.6}, {-14, 16}}));
  // polys.push_back(math::Polygon2d({{-17, 0}, {-18, -6}, {-6, -7.6}, {-5.3, -1.9}}));
  // polys.push_back(math::Polygon2d({{5, -8.7}, {3.7, -12.6}, {14, -20}, {15, -10}}));
  // polys.push_back(math::Polygon2d({{5.6, 13.6}, {3.7, 7.8}, {17, 6.8}, {14.2, 9.6}}));
  // polys.push_back(math::Polygon2d({{16.3, 27.8}, {16.5, 25}, {27.3, 25}, {27.3, 27.9}}));
  // polys.push_back(math::Polygon2d({{-28.4, 3.9}, {-28.3, -3.8}, {-22.6, -4}, {-20.6, 0.5}}));
  // polys.push_back(math::Polygon2d({{-27.8, -23}, {-26.4, -28.5}, {-16.2, -28.3}, {-17.5, -23.2}}));
  // polys.push_back(math::Polygon2d({{23.2, -16.4}, {15.6, -26}, {23.5, -28}, {28.2, -19}}));
  // polys.push_back(math::Polygon2d({{7.1, -1.3}, {6.8, -3.2}, {19, -3.66}, {20.9, -1.329}}));
  // for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
  //   math::Vec2d centre(obstacle[obs_ind].x(), obstacle[obs_ind].y());
  //   std::vector<math::Vec2d> poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), false);
  //   std::vector<math::Vec2d> poly_vertices_inflated = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), true);
  //   // for (int i = 0; i < poly_vertices.size(); i++) {
  //   //     // Obstacles
  //   //     obs.push_back(Vec2f(poly_vertices[i].x(), poly_vertices[i].y()));
  //   // }
  //   polys.push_back(math::Polygon2d(poly_vertices));
  //   polys_inflat.push_back(math::Polygon2d(poly_vertices_inflated));
  // } 
  for (int i = 0; i < polys.size(); i++) {
    polys_inflat_ecbs.push_back(math::Polygon2d({
      {polys[i].points()[0].x() - 0.5, polys[i].points()[0].y() + 0.5}, 
      {polys[i].points()[1].x() - 0.5, polys[i].points()[1].y() - 0.5}, 
      {polys[i].points()[2].x() + 0.5, polys[i].points()[2].y() - 0.5}, 
      {polys[i].points()[3].x() + 0.5, polys[i].points()[3].y() + 0.5}
      }));
  }
  for (int i = 0; i < polys.size(); i++) {
    polys_inflat.push_back(math::Polygon2d({
      {polys[i].points()[0].x() - 0.2, polys[i].points()[0].y() + 0.2}, 
      {polys[i].points()[1].x() - 0.2, polys[i].points()[1].y() - 0.2}, 
      {polys[i].points()[2].x() + 0.2, polys[i].points()[2].y() - 0.2}, 
      {polys[i].points()[3].x() + 0.2, polys[i].points()[3].y() + 0.2}
      }));
  }
  // obs.push_back(Vec2f(60, 60));
  // obs.push_back(Vec2f(-60, 60));
  // obs.push_back(Vec2f(-60, -60));
  // obs.push_back(Vec2f(60, -60));
  iris::IRISProblem iris_problem(2);
  // for(int i = 0; i < polys_inflat.size(); i++) {
  //   Eigen::MatrixXd obs = Eigen::MatrixXd::Zero(2, 4);
  //   obs << polys_inflat[i].points()[0].x(), polys_inflat[i].points()[1].x(), polys_inflat[i].points()[2].x(), polys_inflat[i].points()[3].x(),
  //           polys_inflat[i].points()[0].y(), polys_inflat[i].points()[1].y(), polys_inflat[i].points()[2].y(), polys_inflat[i].points()[3].y();
  //   iris_problem.addObstacle(obs);
  // }
  // add boundary
  Eigen::MatrixXd obs = Eigen::MatrixXd::Zero(2, 4);
  obs << -52, -52, -50, -50,
        -50, 50, 50, -50;
  iris_problem.addObstacle(obs);
  obs << -50, -50, 50, 50,
        50, 52, 52, 50;
  iris_problem.addObstacle(obs);
  obs << 50, 52, 52, 50,
        50, 50, -50, -10;
  iris_problem.addObstacle(obs);
  obs << -50, -50, 50, 50,
        -50, -52, -52, -50;
  iris_problem.addObstacle(obs);

  // env->polygons() = polys;
  env->polygons() = polys_inflat;
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> pointsHyperplane;
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, 2>> pointsVertices;
  int index = 0;
  Timer timer_step;
  // timer_step.reset();
  // for (double i = -50; i < 50; i += 0.2) {
  //     for (double j =  -50; j < 50; j += 0.2) {
  //         Vec2f pos(i, j);  
  //         SeedDecomp2D decomp(pos);
  //         decomp.set_obs(obs);
  //         // Initialize SeedDecomp2D
  //         decomp.set_local_bbox(Vec2f(2, 2));
  //         decomp.dilate(.1);
  //         const auto poly  = decomp.get_polyhedron();
  //         auto linear_cons = LinearConstraint<2>(pos, poly.vs_);
  //         Eigen::MatrixXd matrix_hp(linear_cons.A().rows(), 3);
  //         for (int k = 0; k < linear_cons.A().rows(); k++) {
  //             matrix_hp(k, 0) = linear_cons.A()(k, 0);
  //             matrix_hp(k, 1) = linear_cons.A()(k, 1);
  //             matrix_hp(k, 2) = linear_cons.b()(k, 0);
  //         }
  //         pointsHyperplane.push_back(matrix_hp);
  //         const auto vertices = cal_vertices(poly);
  //         Eigen::MatrixXd matrix_ver(vertices.size(), 2);
  //         for (int k = 0; k < vertices.size(); k++) {
  //             matrix_ver(k, 0) = vertices[k](0);
  //             matrix_ver(k, 1) = vertices[k](1);
  //         }
  //         pointsVertices.push_back(matrix_ver);
  //     }
  // }
  // timer_step.stop();
  // std::cout << "Decomp generation runtime: " << timer_step.elapsedSeconds() << std::endl;
  // Mission
  SwarmPlanning::Mission mission;
  if(!mission.setMission(nh)){
      return -1;
  }

  int qn=mission.qn;
  way_point_pub.resize(qn);
  for(int qi = 0; qi < qn; qi++){
    std::string robot_name = "/robot" + std::to_string(qi+1);
    way_point_pub[qi] = nh.advertise<nav_msgs::Path>(robot_name+"/mpc_predict_all", 1);
  }
  // ROS Parameters
  SwarmPlanning::Param param;
  if(!param.setROSParam(nh)){
      return -1;
  }
  param.setColor(mission.qn);
  // // Submodules
  // std::shared_ptr<DynamicEDTOctomap> distmap_obj;
  // std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
  // std::shared_ptr<Corridor> corridor_obj;
  // std::shared_ptr<MPCPlanner> MPCPlanner_obj;
  // std::shared_ptr<ResultPublisher> resultPublisher_obj;
  visualization::Init(nh, "odom", "/hmfpc_test_vis");

  TrajectoryPoint start, goal;
  FullStates solution;
  FullStates solution_car_like1, solution_diff_drive1, solution_diff_drive2;
  start.x = -10;
  start.y = -10;
  start.theta = 0;
  goal.x = 20;
  goal.y = 20;
  goal.theta = M_PI / 2;
  ros::Rate r(10);

  auto start_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, [&](const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose) {
    double x = pose->pose.pose.position.x;
    double y = pose->pose.pose.position.y;

    double min_distance = DBL_MAX;
    int idx = -1;
    for(int i = 0; i < solution.states.size(); i++) {
      double distance = hypot(solution.states[i].x - x, solution.states[i].y - y);
      if(distance < min_distance) {
        min_distance = distance;
        idx = i;
      }
    }

    start = solution.states[idx];
  });
  Timer timer_total;
  double start_time, current_time;
  int count = 0;
  while(ros::ok()) {
    // Submodules
    std::shared_ptr<DynamicEDTOctomap> distmap_obj;
    std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
    std::shared_ptr<Corridor> corridor_obj;
    std::shared_ptr<MPCPlanner> MPCPlanner_obj;
    std::shared_ptr<ResultPublisher> resultPublisher_obj;
    // if (!has_path) {
    for (int i = 0; i < polys.size(); i++) {
      visualization::PlotPolygon(polys[i], 0.5, visualization::Color::White, i, "Obstacle"+  std::to_string(i));
    }    
    visualization::Trigger();   
    timer_total.reset();
    ROS_INFO("Multi-robot Trajectory Planning");

    // Step 1: Plan Initial Trajectory
    timer_step.reset();
    {
        initTrajPlanner_obj.reset(new ECBSPlanner(polys_inflat_ecbs, mission, param));
        // initTrajPlanner_obj.reset(new ECBSPlanner(polys_inflat, mission, param));
        if (!initTrajPlanner_obj.get()->update(param.log)) {
            return -1;
        }
    }
    timer_step.stop();
    ROS_INFO_STREAM("ECBS Planner runtime: " << timer_step.elapsedSeconds());
    for (int i = 0; i < initTrajPlanner_obj->initTraj.size(); i++) {
      DrawCorasePathRivz(initTrajPlanner_obj->initTraj[i], i);
    }
    
    // Step 2: Generate Safe Corridor
    timer_step.reset();
    // {
    corridor_obj.reset(new Corridor(initTrajPlanner_obj, mission, param));
    //     if (!corridor_obj.get()->update(param.log)) {
    //         return -1;
    //     }
    // }
    std::vector<std::vector<std::vector<double>>> plan_sert = 
      InsertPath(initTrajPlanner_obj->initTraj.size(), initTrajPlanner_obj->initTraj[0].size() - 1, initTrajPlanner_obj->initTraj);
    std::vector<heterogeneous_formation_controller::Constraints> constraint_set;
    constraint_set.resize(initTrajPlanner_obj->initTraj.size());
    // generate decomp
    // for (int robot_ind = 0; robot_ind < initTrajPlanner_obj->initTraj.size(); robot_ind++) {
    //   for(size_t i = 0; i < 5 * (initTrajPlanner_obj->initTraj[0].size() - 1) + 1; i++) {
    //       int decomp_ind = int((plan_sert[robot_ind][i][0] + 50) / 0.2 * 500 + int((plan_sert[robot_ind][i][1] + 50) / 0.2));
    //       auto decomp_matrix = pointsVertices[decomp_ind];
    //       std::vector<math::Vec2d> decomp_vert_set;
    //       for (int v_ind = 0; v_ind < decomp_matrix.rows(); v_ind++) {
    //         math::Vec2d decomp_vert(decomp_matrix(v_ind, 0), decomp_matrix(v_ind, 1));
    //         decomp_vert_set.push_back(decomp_vert);
    //       }          
    //       auto color = visualization::Color::Green;
    //       color.set_alpha(0.1);
    //       visualization::PlotPolygon(math::Polygon2d(decomp_vert_set), 0.05, color, i, "Robot" + std::to_string(robot_ind) + " :Corridor " + std::to_string(i));
    //       // visualization::PlotPolygon(math::Polygon2d({points_}), 0.05, color, i, "Convex Region" + std::to_string(j));
    //   }
    // } 
    // visualization::Trigger();     
    iris::IRISOptions options;
    // for (int robot_ind = 0; robot_ind < initTrajPlanner_obj->initTraj.size(); robot_ind++) {
    //   for(size_t i = 0; i < 5 * (initTrajPlanner_obj->initTraj[0].size() - 1) + 1; i++) {
    //     iris_problem.setSeedPoint(Eigen::Vector2d(plan_sert[robot_ind][i][0], plan_sert[robot_ind][i][1]));
    //     iris::IRISRegion region = inflate_region(iris_problem, options);
    //     auto points = region.polyhedron.generatorPoints();
    //     std::vector<math::Vec2d> points_;
    //     for (const auto& pts : points) {
    //       math::Vec2d pts_temp(pts[0], pts[1]);
    //       points_.push_back(pts_temp);
    //     }
    //     auto color = visualization::Color::Green;
    //     color.set_alpha(0.1);
    //     visualization::PlotPolygon(math::Polygon2d({points_}), 0.05, color, i, "Convex Region"+  std::to_string(robot_ind) + "_" + std::to_string(i));
    //   }
    // }
    visualization::Trigger();
    timer_step.stop();
    ROS_INFO_STREAM("IRIS runtime: " << timer_step.elapsedSeconds());
    for (int robot_ind = 0; robot_ind < initTrajPlanner_obj->initTraj.size(); robot_ind++) {
      TrajectoryPoint start;
      start.x = plan_sert[robot_ind][0][0];
      start.y = plan_sert[robot_ind][0][1];
      start.theta = plan_sert[robot_ind][0][2];
      goal.x = plan_sert[robot_ind][initTrajPlanner_obj->initTraj[robot_ind].size() - 1][0];
      goal.y = plan_sert[robot_ind][initTrajPlanner_obj->initTraj[robot_ind].size() - 1][1];
      goal.theta = plan_sert[robot_ind][initTrajPlanner_obj->initTraj[robot_ind].size() - 1][2];
      constraint_set[robot_ind].start = start;
      constraint_set[robot_ind].goal = goal;
      constraint_set[robot_ind].corridor_lb.setConstant(5 * (initTrajPlanner_obj->initTraj[robot_ind].size() - 1) + 1, 2, -inf);
      constraint_set[robot_ind].corridor_ub.setConstant(5 * (initTrajPlanner_obj->initTraj[robot_ind].size() - 1) + 1, 2, inf);
      iris::IRISOptions options;
      for(size_t i = 0; i < 5 * (initTrajPlanner_obj->initTraj[0].size() - 1) + 1; i++) {
          math::AABox2d box;
          if (!env->GenerateCorridorBox(0.0, plan_sert[robot_ind][i][0], plan_sert[robot_ind][i][1], 0.1, box)) {
            ROS_ERROR("the corridor box indexed at %ld generation failed!", i);
            return false;
          }
          auto color = visualization::Color::Green;
          color.set_alpha(0.1);
          // visualization::PlotPolygon(math::Polygon2d(math::Box2d(box)), 0.05, color, i, "Robot" + std::to_string(robot_ind) + " :Corridor " + std::to_string(i));
          // visualization::PlotPolygon(math::Polygon2d({points_}), 0.05, color, i, "Convex Region" + std::to_string(j));
          constraint_set[robot_ind].corridor_lb(i, 0) = box.min_x();
          constraint_set[robot_ind].corridor_lb(i, 1) = box.min_y();
          constraint_set[robot_ind].corridor_ub(i, 0) = box.max_x();
          constraint_set[robot_ind].corridor_ub(i, 1) = box.max_y();
      }
    }
    std::vector<std::pair<double, double>> goal_sets;
    for (int robot_ind = 0; robot_ind < initTrajPlanner_obj->initTraj.size(); robot_ind++) {
      double goal_x = initTrajPlanner_obj->mission.goalState[robot_ind][0];
      double goal_y = initTrajPlanner_obj->mission.goalState[robot_ind][1];
      goal_sets.push_back(std::make_pair(goal_x, goal_y));
    } 
    // visualization::Trigger();
    timer_step.stop();
    ROS_INFO_STREAM("Safe Corridor runtime: " << timer_step.elapsedSeconds());
    
    // // Step 3: Formulate NLP problem and solving it to generate trajectory for the robot team
    std::vector<std::vector<std::vector<double>>>& plan_set = plan;
    std::vector<double> theta_set;
    timer_step.reset();
    {
        MPCPlanner_obj.reset(new MPCPlanner(constraint_set, corridor_obj, initTrajPlanner_obj, mission, param, theta_set));
        if (!MPCPlanner_obj.get()->update(param.log, plan_set)) {
            return -1;
        }
    }
    timer_step.stop();
    ROS_INFO_STREAM("Trajectory Optimization runtime: " << timer_step.elapsedSeconds());
    timer_total.stop();
    ROS_INFO_STREAM("Overall runtime: " << timer_total.elapsedSeconds());
    auto time_sets = CalAvgFlowtime(plan_set);
    ROS_INFO_STREAM("Average flowtime: " << 0.32 * time_sets[0]);
    ROS_INFO_STREAM("Makespan: " << 0.32 * time_sets[1]);
    
    // delete_yaml("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/fg_plan.yaml");
    for (int i = 0; i < plan_set.size(); i++) {
      DrawFGTrajRivz(plan_set[i], i, 5 * (initTrajPlanner_obj->initTraj[0].size() - 1) + 1);
    }
    double terminal_error = 0.0;
    double terminal_error_ang = 0.0;
    int ter_ind = 0;
    for (ter_ind = 0; ter_ind != plan_set[0].size(); ter_ind++) {
      if (plan_set[0][ter_ind][0] == 0) 
        break;
    }
    ter_ind = ter_ind - 1;
    // theta_set = {-0.78539789999291676, -0.78539789999291676, -0.78539789999291676, -1.5707963267948966, -1.5707963267948966,
    // -0.78539789999291676, -0.78539789999291676, -0.78539789999291676, -0.78539789999291676, -1.5707963267948966};
    for (int i = 0; i < plan_set.size(); i++) {
      terminal_error_ang += abs(plan_set[i][ter_ind][2] - theta_set[i]);
    }
    for (int i = 0; i < plan_set.size(); i++) {
      terminal_error += sqrt(pow(plan_set[i][ter_ind][0] - goal_sets[i].first, 2) + pow(plan_set[i][ter_ind][1] - goal_sets[i].second, 2));
    }
    terminal_error /= goal_sets.size();
    terminal_error_ang /= goal_sets.size();
    ROS_WARN("TERMINAL ERROR: %f", terminal_error);
    ROS_WARN("TERMINAL ANGLE ERROR: %f", terminal_error_ang);
    std::vector<double> new_vector;
    new_vector.push_back(4);
    new_vector.push_back(terminal_error);
    new_vector.push_back(terminal_error_ang);
    writeTrajectoryToYAML_FG(plan_set, 5 * (initTrajPlanner_obj->initTraj[0].size() - 1) + 1, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/ras_demo/formation_generation/fg_16.yaml");  
    // writeVectorToYAML(new_vector, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/fg_terminal.yaml");
    // Plot Planning Result
    resultPublisher_obj.reset(new ResultPublisher(nh, MPCPlanner_obj, corridor_obj, initTrajPlanner_obj, mission, param));
    resultPublisher_obj->plot(param.log);

    start_time = ros::Time::now().toSec();
    has_path = true;
      // }
      // if(has_path) {
      //     // Publish Swarm Trajectory
      //     current_time = ros::Time::now().toSec() - start_time;
      //     resultPublisher_obj.get()->update(current_time);
      //     resultPublisher_obj.get()->publish();
      // }
    // if(!planner_->Plan(solution, start, goal, iris_problem, solution)) {
    //   break;
    // }
    // writeTrajectoryToYAML(solution_car_like1, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/traj_car1.yaml");
    // DrawTrajectoryRviz(solution, config_, 0, path_pub);
    ros::spinOnce();
    r.sleep();
    // ros::spin();
    if (count++ == 29)
      break;
  }
ros::spin();
return 0;
}
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
  visualization::Color::Blue,
  visualization::Color::Cyan,
  visualization::Color::Green,
  // visualization::Color::Grey,
  visualization::Color::Yellow,
  visualization::Color::Red,
  // visualization::Color::White,
  visualization::Color::Red,
  visualization::Color::Blue,
  // visualization::Color::Black,
  visualization::Color::Blue,
  visualization::Color::Cyan,
  visualization::Color::Cyan,
  // visualization::Color::Grey,
  visualization::Color::Green,
  // visualization::Color::Black,
  visualization::Color::Green,
  visualization::Color::Yellow,
  visualization::Color::Yellow,
  // visualization::Color::Grey,
  visualization::Color::Magenta,
  visualization::Color::Red,
  // visualization::Color::Grey,
  visualization::Color::Magenta,
  visualization::Color::Red,
  // visualization::Color::White,
  visualization::Color::Yellow,
  visualization::Color::Yellow,
  // visualization::Color::Black,
  visualization::Color::Blue
};

void DrawFGTrajRivz(const std::vector<vector<double>> traj, int robot_index, int traj_length) {
  std::vector<double> xs, ys;
  const std::string robot_name = "Footprint_" + std::to_string(robot_index);
  for (int i = 0; i < traj_length; i++) {
    xs.push_back(traj[i][0]);
    ys.push_back(traj[i][1]);
  }
  visualization::Plot(xs, ys, 0.4, colors[robot_index], robot_index, robot_name);
  visualization::Trigger();
}


int main(int argc, char **argv) {
    
  // 它创建的节点句柄将使用波浪线作为命名空间，这意味着它将访问和处理与当前节点相关的参数和话题的私有命名空间。
  // 这对于在ROS中编写节点时非常有用，因为它允许您将参数和话题隔离在节点的私有命名空间中，以避免与其他节点的
  // 参数和话题发生冲突。
  ros::init(argc, argv, "hmfpc_test_node");
  ros::NodeHandle nh( "~" );
  visualization::Init(nh, "map", "/hmfpc_test_vis");
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
  std::vector<math::Polygon2d> polys_inflat, polys;
  ros::Rate r(10);
  Timer timer_total;
  double start_time, current_time;
  while(ros::ok()) {
    // delete_yaml("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/fg_plan.yaml");
    auto plan_set =  generate_traj_fg("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/fg_plan_demo.yaml", 15);

    for (int i = 0; i < plan_set.size(); i++) {
      DrawFGTrajRivz(plan_set[i], i, 466);
    }
    ros::spin();
    r.sleep();
    // ros::spin();
    return 0;
  }
}
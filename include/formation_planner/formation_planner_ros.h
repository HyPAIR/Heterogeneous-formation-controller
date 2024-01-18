// //
// // Created by weijian on 17/11/23.
// //

// #ifndef SRC_formation_planner_ROS_H
// #define SRC_formation_planner_ROS_H

// // abstract class from which our plugin inherits
// #include <nav_core/base_local_planner.h>
// #include <nav_core/base_global_planner.h>
// #include <ros/ros.h>
// #include <tf2_ros/buffer.h>
// #include <costmap_2d/costmap_2d_ros.h>
// #include <base_local_planner/odometry_helper_ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Twist.h>

// #include "formation_planner.h"

// using namespace std;

// namespace formation_planner {

// struct FormationPlannerROSConfigConfig {
//   double goal_xy_tolerance = 0.5;
//   double goal_yaw_tolerance = 0.2;

//   std::string odom_topic = "odom";
//   std::string global_frame = "map";
// };

// class FormationPlannerROSConfig : public nav_core::BaseLocalPlanner {
// public:

//   FormationPlannerROSConfig();
//   FormationPlannerROSConfig(std::string name, tf2_ros::Buffer* tf,
//                       costmap_2d::Costmap2DROS* costmap_ros);

//   ~FormationPlannerROSConfig();

//   void initialize(std::string name, tf2_ros::Buffer* tf,
//                   costmap_2d::Costmap2DROS* costmap_ros);

//   bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

//   bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

//   bool isGoalReached();
// private:
//   costmap_2d::Costmap2DROS* costmap_ros_;
//   base_local_planner::OdometryHelperRos odom_helper_;
//   tf2_ros::Buffer* tf_;
//   bool initialized_;
//   std::string name_;

//   FormationPlannerROSConfigConfig config_;

//   std::shared_ptr<formation_planner::PlannerConfig> planner_config_;
//   std::shared_ptr<formation_planner::Environment> env_;
//   std::shared_ptr<formation_planner::LiomLocalPlanner> planner_;
//   TrajectoryPoint start_state_, goal_state_;
//   ros::Publisher path_pub_;
//   FullStates solution_;
//   bool goal_reached_ = false;

//   void ReadParameters(const ros::NodeHandle &nh);
// };
// };

// #endif //SRC_formation_planner_ROS_H
//
// Created by weijian on 17/11/23.
//

#ifndef SRC_formation_planner_ROS_H
#define SRC_formation_planner_ROS_H

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include "formation_planner.h"

using namespace std;

namespace formation_planner {

struct FormationPlannerROSConfig {
  double goal_xy_tolerance = 0.5;
  double goal_yaw_tolerance = 0.2;

  std::string odom_topic = "odom";
  std::string global_frame = "map";
};

class FormationPlannerROSConfig : public nav_core::BaseGlobalPlanner {
public:

  FormationPlannerROSConfig();
  FormationPlannerROSConfig(std::string name,
                      costmap_2d::Costmap2DROS* costmap_ros);

  ~FormationPlannerROSConfig();

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);


private:
  costmap_2d::Costmap2DROS* costmap_ros_;
  base_local_planner::OdometryHelperRos odom_helper_;
  tf2_ros::Buffer* tf_;
  bool initialized_;
  std::string name_;

  FormationPlannerROSConfig config_;

  std::shared_ptr<formation_planner::PlannerConfig> planner_config_;
  std::shared_ptr<formation_planner::Environment> env_;
  std::shared_ptr<formation_planner::LiomLocalPlanner> planner_;
  TrajectoryPoint start_state_, goal_state_;
  ros::Publisher path_pub_;
  FullStates solution_;
  bool goal_reached_ = false;

  void ReadParameters(const ros::NodeHandle &nh);
};
};

#endif //SRC_formation_planner_ROS_H

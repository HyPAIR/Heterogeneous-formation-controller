// //
// // Created by weijian on 17/11/23.
// //

// #ifndef SRC_heterogeneous_formation_controller_ROS_H
// #define SRC_heterogeneous_formation_controller_ROS_H

// // abstract class from which our plugin inherits
// #include <nav_core/base_local_planner.h>
// #include <nav_core/base_global_planner.h>
// #include <ros/ros.h>
// #include <tf2_ros/buffer.h>
// #include <costmap_2d/costmap_2d_ros.h>
// #include <base_local_planner/odometry_helper_ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Twist.h>

// #include "heterogeneous_formation_controller.h"

// using namespace std;

// namespace heterogeneous_formation_controller {

// struct hmfpcLocalPlannerROSConfig {
//   double goal_xy_tolerance = 0.5;
//   double goal_yaw_tolerance = 0.2;

//   std::string odom_topic = "odom";
//   std::string global_frame = "map";
// };

// class hmfpcLocalPlannerROS : public nav_core::BaseLocalPlanner {
// public:

//   hmfpcLocalPlannerROS();
//   hmfpcLocalPlannerROS(std::string name, tf2_ros::Buffer* tf,
//                       costmap_2d::Costmap2DROS* costmap_ros);

//   ~hmfpcLocalPlannerROS();

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

//   hmfpcLocalPlannerROSConfig config_;

//   std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> planner_config_;
//   std::shared_ptr<heterogeneous_formation_controller::Environment> env_;
//   std::shared_ptr<heterogeneous_formation_controller::hmfpcLocalPlanner> planner_;
//   TrajectoryPoint start_state_, goal_state_;
//   ros::Publisher path_pub_;
//   FullStates solution_;
//   bool goal_reached_ = false;

//   void ReadParameters(const ros::NodeHandle &nh);
// };
// };

// #endif //SRC_heterogeneous_formation_controller_ROS_H
//
// Created by weijian on 17/11/23.
//

#ifndef SRC_heterogeneous_formation_controller_ROS_H
#define SRC_heterogeneous_formation_controller_ROS_H

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include "heterogeneous_formation_controller.h"

using namespace std;

namespace heterogeneous_formation_controller {

struct hmfpcLocalPlannerROSConfig {
  double goal_xy_tolerance = 0.5;
  double goal_yaw_tolerance = 0.2;

  std::string odom_topic = "odom";
  std::string global_frame = "map";
};

class hmfpcLocalPlannerROS : public nav_core::BaseGlobalPlanner {
public:

  hmfpcLocalPlannerROS();
  hmfpcLocalPlannerROS(std::string name,
                      costmap_2d::Costmap2DROS* costmap_ros);

  ~hmfpcLocalPlannerROS();

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);


private:
  costmap_2d::Costmap2DROS* costmap_ros_;
  base_local_planner::OdometryHelperRos odom_helper_;
  tf2_ros::Buffer* tf_;
  bool initialized_;
  std::string name_;

  hmfpcLocalPlannerROSConfig config_;

  std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> planner_config_;
  std::shared_ptr<heterogeneous_formation_controller::Environment> env_;
  std::shared_ptr<heterogeneous_formation_controller::hmfpcLocalPlanner> planner_;
  TrajectoryPoint start_state_, goal_state_;
  ros::Publisher path_pub_;
  FullStates solution_;
  bool goal_reached_ = false;

  void ReadParameters(const ros::NodeHandle &nh);
};
};

#endif //SRC_heterogeneous_formation_controller_ROS_H

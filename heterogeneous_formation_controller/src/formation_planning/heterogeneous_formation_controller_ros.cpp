// //
// // Created by weijian on 17/11/23.
// //
// #include "heterogeneous_formation_controller/heterogeneous_formation_controller_ros.h"
// #include "heterogeneous_formation_controller/visualization/plot.h"
// #include "heterogeneous_formation_controller/math/math_utils.h"
// #include <tf2_ros/transform_listener.h>
// #include <costmap_2d/costmap_2d_ros.h>
// #include <base_local_planner/goal_functions.h>
// #include <tf/transform_datatypes.h>
// #include <tf2/utils.h>
// #include <pluginlib/class_list_macros.h>
// #include "heterogeneous_formation_controller/yaml_all.h"

// PLUGINLIB_EXPORT_CLASS(heterogeneous_formation_controller::hmfpcLocalPlannerROS, nav_core::BaseLocalPlanner)

// namespace heterogeneous_formation_controller {

// hmfpcLocalPlannerROS::hmfpcLocalPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

// hmfpcLocalPlannerROS::hmfpcLocalPlannerROS(std::string name, tf2_ros::Buffer *tf,
//                                          costmap_2d::Costmap2DROS *costmap_ros)
//     : costmap_ros_(NULL), tf_(NULL), initialized_(false) {
//   initialize(name, tf, costmap_ros);
// }

// hmfpcLocalPlannerROS::~hmfpcLocalPlannerROS() {}
// void DrawTrajectoryRvizRos(const FullStates sol_traj,std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config,
//       int robot_index, ros::Publisher path_pub) {
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
//     auto color = robot_index > 0 ? visualization::Color::Red : visualization::Color::Green;
//     // color.set_alpha(0.4);
//     visualization::PlotPolygon(math::Polygon2d(box), 0.05, color, i, robot_name);
//   }
//   path_pub.publish(msg);
//   visualization::Trigger();
// }
// void hmfpcLocalPlannerROS::ReadParameters(const ros::NodeHandle &nh) {
//   nh.param("goal_xy_tolerance", config_.goal_xy_tolerance, config_.goal_xy_tolerance);
//   nh.param("goal_yaw_tolerance", config_.goal_yaw_tolerance, config_.goal_yaw_tolerance);

//   nh.param("odom_topic", config_.odom_topic, config_.odom_topic);
//   nh.param("global_frame", config_.global_frame, config_.global_frame);

//   nh.param("coarse_xy_resolution", planner_config_->xy_resolution, planner_config_->xy_resolution);
//   nh.param("coarse_theta_resolution", planner_config_->theta_resolution, planner_config_->theta_resolution);
//   nh.param("coarse_step_size", planner_config_->step_size, planner_config_->step_size);
//   nh.param("coarse_next_node_num", planner_config_->next_node_num, planner_config_->next_node_num);
//   nh.param("coarse_grid_xy_resolution", planner_config_->grid_xy_resolution, planner_config_->grid_xy_resolution);
//   nh.param("coarse_forward_penalty", planner_config_->forward_penalty, planner_config_->forward_penalty);
//   nh.param("coarse_backward_penalty", planner_config_->backward_penalty, planner_config_->backward_penalty);
//   nh.param("coarse_gear_change_penalty", planner_config_->gear_change_penalty, planner_config_->gear_change_penalty);
//   nh.param("coarse_steering_penalty", planner_config_->steering_penalty, planner_config_->steering_penalty);
//   nh.param("coarse_steering_change_penalty", planner_config_->steering_change_penalty, planner_config_->steering_change_penalty);

//   nh.param("min_waypoints", planner_config_->min_nfe, planner_config_->min_nfe);
//   nh.param("time_step", planner_config_->time_step, planner_config_->time_step);
//   nh.param("corridor_max_iter", planner_config_->corridor_max_iter, planner_config_->corridor_max_iter);
//   nh.param("corridor_incremental_limit", planner_config_->corridor_incremental_limit, planner_config_->corridor_incremental_limit);
//   nh.param("weight_a", planner_config_->opti_w_a, planner_config_->opti_w_a);
//   nh.param("weight_omega", planner_config_->opti_w_omega, planner_config_->opti_w_omega);
//   nh.param("max_iter", planner_config_->opti_inner_iter_max, planner_config_->opti_inner_iter_max);
//   nh.param("infeasible_penalty", planner_config_->opti_w_penalty0, planner_config_->opti_w_penalty0);
//   nh.param("infeasible_tolerance", planner_config_->opti_varepsilon_tol, planner_config_->opti_varepsilon_tol);

//   nh.param("front_hang_length", planner_config_->vehicle.front_hang_length, planner_config_->vehicle.front_hang_length);
//   nh.param("wheel_base", planner_config_->vehicle.wheel_base, planner_config_->vehicle.wheel_base);
//   nh.param("rear_hang_length", planner_config_->vehicle.rear_hang_length, planner_config_->vehicle.rear_hang_length);
//   nh.param("width", planner_config_->vehicle.width, planner_config_->vehicle.width);

//   nh.param("max_velocity", planner_config_->vehicle.max_velocity, planner_config_->vehicle.max_velocity);
//   nh.param("min_velocity", planner_config_->vehicle.min_velocity, planner_config_->vehicle.min_velocity);
//   nh.param("max_acceleration", planner_config_->vehicle.max_acceleration, planner_config_->vehicle.max_acceleration);
//   nh.param("max_steering", planner_config_->vehicle.phi_max, planner_config_->vehicle.phi_max);
//   nh.param("max_steering_rate", planner_config_->vehicle.omega_max, planner_config_->vehicle.omega_max);
//   nh.param("n_disc", planner_config_->vehicle.n_disc, planner_config_->vehicle.n_disc);
// }

// // Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
// void hmfpcLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf,
//                                      costmap_2d::Costmap2DROS *costmap_ros) {
//   if (!initialized_) {
//     tf_ = tf;
//     costmap_ros_ = costmap_ros;
//     name_ = name;
//     initialized_ = true;

//     planner_config_ = std::make_shared<heterogeneous_formation_controller::PlannerConfig>();
//     planner_config_->vehicle.InitializeDiscs();

//     env_ = std::make_shared<heterogeneous_formation_controller::Environment>(planner_config_);
//     planner_ = std::make_shared<heterogeneous_formation_controller::hmfpcLocalPlanner>(planner_config_, env_);

//     ros::NodeHandle nh("~/" + name_);
//     ReadParameters(nh);

//     visualization::Init(nh, config_.global_frame, "hmfpc_markers");
//     odom_helper_.setOdomTopic(config_.odom_topic);

//     path_pub_ = nh.advertise<nav_msgs::Path>("local_path", 1);
//   }
// }

// bool hmfpcLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) {
//   if (!initialized_) {
//     ROS_ERROR("This planner has not been initialized");
//     return false;
//   }

//   std::vector<math::Pose> path;
//   path.reserve(orig_global_plan.size());
//   for(auto &pose: orig_global_plan) {
//     path.emplace_back(pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation));
//   }
//   planner_->set_global_path(path);

//   geometry_msgs::PoseStamped goal_pose;
//   if(!base_local_planner::getGoalPose(*tf_, orig_global_plan, config_.global_frame, goal_pose)) {
//     ROS_ERROR("get goal pose failed");
//     return false;
//   }

//   goal_state_.x = goal_pose.pose.position.x;
//   goal_state_.y = goal_pose.pose.position.y;
//   goal_state_.theta = math::NormalizeAngle(tf2::getYaw(goal_pose.pose.orientation));
//   goal_reached_ = false;

//   return true;
// }

// bool hmfpcLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
//   FullStates solution, solution_car;
//   FullStates solution_car_like1, solution_diff_drive1, solution_diff_drive2;
//   double max_error1, coarse_path_time, solve_time_leader, solve_time_diff1, solve_time_diff2, solution_car_like;
//   TrajectoryPoint start, goal;
//   start.x = -15;
//   start.y = -15;
//   start.theta = M_PI / 2;
//   goal.x = 15;
//   goal.y = 15;
//   goal.theta = M_PI / 2;
//   if (!initialized_) {
//     ROS_ERROR("This planner has not been initialized");
//     return false;
//   }

//   goal_reached_ = false;
//   env_->UpdateCostmapObstacles(costmap_ros_->getCostmap());
//   iris::IRISProblem iris_problem(2);
//   Eigen::MatrixXd obs = Eigen::MatrixXd::Zero(2, 4);
//   for(int i = 0; i < env_->points().size(); i++) {
//     obs << env_->points()[i].x() + 0.25, env_->points()[i].x() - 0.25, env_->points()[i].x() - 0.25, env_->points()[i].x() + 0.25,
//            env_->points()[i].y() + 0.25, env_->points()[i].y() + 0.25, env_->points()[i].y() - 0.25, env_->points()[i].y() - 0.25;
//     iris_problem.addObstacle(obs);
//   }

//   geometry_msgs::PoseStamped current_pose;
//   if(!costmap_ros_->getRobotPose(current_pose)) {
//     ROS_ERROR("failed to get robot pose");
//     return false;
//   }
//   start_state_.x = current_pose.pose.position.x;
//   start_state_.y = current_pose.pose.position.y;
//   start_state_.theta = math::NormalizeAngle(tf2::getYaw(current_pose.pose.orientation));

//   geometry_msgs::PoseStamped current_vel;
//   odom_helper_.getRobotVel(current_vel);
//   start_state_.v = current_vel.pose.position.x;

//   if(std::fabs(start_state_.v) > 1e-3) {
//     double dtheta = tf2::getYaw(current_vel.pose.orientation);
//     start_state_.phi = atan(dtheta / start_state_.v * planner_config_->vehicle.wheel_base);
//   }

//   if(solution_.states.size() > 1) {
//     // assume control inputs reached desired values
//     start_state_.a = solution_.states[1].a;
//     start_state_.omega = solution_.states[1].omega;
//   }
//   start_state_.x = -60;
//   start_state_.y = 36;
//   start_state_.theta = 0;
//   start_state_.v = 0;
//   start_state_.a = 0;
//   start_state_.phi = 0;
//   start_state_.omega = 0;
  
//   // goal_state_.x = 0;
//   // goal_state_.y = 0;
//   // goal_state_.theta = 0;
//   // goal_state_.v = 0;
//   // goal_state_.a = 0;
//   // goal_state_.phi = 0;
//   // goal_state_.omega = 0;

//   if(std::fabs(start_state_.x - goal_state_.x) < config_.goal_xy_tolerance && std::fabs(start_state_.y - goal_state_.y) < config_.goal_xy_tolerance && std::fabs(math::AngleDiff(start_state_.theta, goal_state_.theta)) < config_.goal_yaw_tolerance) {
//     goal_reached_ = true;
//     solution_ = decltype(solution_)();
//     return true;
//   }
//   bool plan_result = planner_->Plan(solution_, start_state_, goal_state_, iris_problem, solution_, coarse_path_time, solve_time_leader);
//   if(!plan_result) {
//     ROS_ERROR("local plan failed");
//     return false;
//   }
//   double solve_time_car = 0.0;
//   if (!planner_->Plan_car_like(solution_, 2.016733665, solution_car_like1, solve_time_car)) {
//   }
//   double infeasible = 0.0;
//   if (!planner_->Plan_car_like_replan(solution_car_like1, solution_car, solution_car, 1, infeasible, solution_car_like)) {
//   }
//   auto traj_leader = fulltotraj(solution_);
//   auto traj_fol_diff1 = generate_ref_traj_diff(traj_leader, hypot(-18-(-20), -21.0083668325-(-20)), atan2(21.0083668325-20, 20-18));

//   if(!planner_->Plan_diff_drive(traj_fol_diff1, solution_diff_drive1, start, goal, solution_diff_drive1, 1, max_error1, solve_time_diff1)) {
//   }
//   ros::NodeHandle nh;
//   visualization::Init(nh, "odom", "/hmfpc_test_vis");
//   ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/hmfpc_test_path", 1, false);
//   ros::Publisher path_pub_diff1 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_diff1", 1, false);
//   ros::Publisher path_pub_diff2 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_diff2", 1, false);
//   ros::Publisher path_pub_car1 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_car1", 1, false);
//   DrawTrajectoryRvizRos(solution_, planner_config_, 0, path_pub);
//   DrawTrajectoryRvizRos(solution_car, planner_config_, 1, path_pub_car1);
//   DrawTrajectoryRvizRos(solution_diff_drive1, planner_config_, 2, path_pub_diff1);
//   nav_msgs::Path msg;
//   msg.header.frame_id = config_.global_frame;
//   msg.header.stamp = ros::Time::now();
//   for(size_t i = 0; i < solution_.states.size(); i++) {
//     geometry_msgs::PoseStamped pose;
//     pose.header = msg.header;
//     pose.pose.position.x = solution_.states[i].x;
//     pose.pose.position.y = solution_.states[i].y;
//     pose.pose.orientation = tf::createQuaternionMsgFromYaw(solution_.states[i].theta);
//     msg.poses.push_back(pose);
//   }

//   path_pub_.publish(msg);

//   if(solution_.states.size() < 2) {
//     cmd_vel.linear.x = 0;
//     cmd_vel.angular.z = 0;
//   }

//   cmd_vel.linear.x = solution_.states[1].v;
//   cmd_vel.angular.z = solution_.states[1].phi;
//   return true;
// }

// bool hmfpcLocalPlannerROS::isGoalReached() {
//   if (!initialized_) {
//     ROS_ERROR("This planner has not been initialized");
//     return false;
//   }

//   return goal_reached_;
// }
// }
//
// Created by weijian on 17/11/23.
//
#include "heterogeneous_formation_controller/heterogeneous_formation_controller_ros.h"
#include "heterogeneous_formation_controller/visualization/plot.h"
#include "heterogeneous_formation_controller/math/math_utils.h"
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/goal_functions.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>
#include <pluginlib/class_list_macros.h>
#include "heterogeneous_formation_controller/yaml_all.h"
void writeVectorToYAML(const std::vector<double>& data, const std::string& file_path) {
    try {
        // 尝试加载已有的数据
        YAML::Node existing_data;
        std::ifstream file_in(file_path);
        if (file_in) {
            existing_data = YAML::Load(file_in);
            file_in.close();
        }

        // 将新的向量追加到数据中
        YAML::Node new_data;
        for (const auto& value : data) {
            new_data.push_back(value);
        }
        existing_data.push_back(new_data);

        // 以追加模式打开文件，并将新数据写入文件
        std::ofstream file_out(file_path, std::ofstream::out | std::ofstream::trunc);
        file_out << existing_data;
        file_out.close();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}
PLUGINLIB_EXPORT_CLASS(heterogeneous_formation_controller::hmfpcLocalPlannerROS, nav_core::BaseGlobalPlanner)

namespace heterogeneous_formation_controller {

hmfpcLocalPlannerROS::hmfpcLocalPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

hmfpcLocalPlannerROS::hmfpcLocalPlannerROS(std::string name,
                                         costmap_2d::Costmap2DROS *costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false) {
  initialize(name, costmap_ros);
}

hmfpcLocalPlannerROS::~hmfpcLocalPlannerROS() {}
void DrawTrajectoryRvizRos(const FullStates sol_traj,std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config,
      int robot_index, ros::Publisher path_pub) {
  const std::string robot_name = "Footprint_" + std::to_string(robot_index);
  for(int i = 0; i < 1e3; i++) {
    visualization::Delete(i, robot_name);
  }
  visualization::Trigger();

  nav_msgs::Path msg;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();
  for(size_t i = 0; i < sol_traj.states.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = sol_traj.states[i].x;
    pose.pose.position.y = sol_traj.states[i].y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(sol_traj.states[i].theta);
    msg.poses.push_back(pose);

    auto box = config->vehicle.GenerateBox(sol_traj.states[i].pose());
    auto color = robot_index > 0 ? visualization::Color::Green : visualization::Color::Red;
    // color.set_alpha(0.4);
    visualization::PlotPolygon(math::Polygon2d(box), 0.2, color, i, robot_name);
  }
  path_pub.publish(msg);
  visualization::Trigger();
}
void hmfpcLocalPlannerROS::ReadParameters(const ros::NodeHandle &nh) {
  nh.param("goal_xy_tolerance", config_.goal_xy_tolerance, config_.goal_xy_tolerance);
  nh.param("goal_yaw_tolerance", config_.goal_yaw_tolerance, config_.goal_yaw_tolerance);

  nh.param("odom_topic", config_.odom_topic, config_.odom_topic);
  nh.param("global_frame", config_.global_frame, config_.global_frame);

  nh.param("coarse_xy_resolution", planner_config_->xy_resolution, planner_config_->xy_resolution);
  nh.param("coarse_theta_resolution", planner_config_->theta_resolution, planner_config_->theta_resolution);
  nh.param("coarse_step_size", planner_config_->step_size, planner_config_->step_size);
  nh.param("coarse_next_node_num", planner_config_->next_node_num, planner_config_->next_node_num);
  nh.param("coarse_grid_xy_resolution", planner_config_->grid_xy_resolution, planner_config_->grid_xy_resolution);
  nh.param("coarse_forward_penalty", planner_config_->forward_penalty, planner_config_->forward_penalty);
  nh.param("coarse_backward_penalty", planner_config_->backward_penalty, planner_config_->backward_penalty);
  nh.param("coarse_gear_change_penalty", planner_config_->gear_change_penalty, planner_config_->gear_change_penalty);
  nh.param("coarse_steering_penalty", planner_config_->steering_penalty, planner_config_->steering_penalty);
  nh.param("coarse_steering_change_penalty", planner_config_->steering_change_penalty, planner_config_->steering_change_penalty);

  nh.param("min_waypoints", planner_config_->min_nfe, planner_config_->min_nfe);
  nh.param("time_step", planner_config_->time_step, planner_config_->time_step);
  nh.param("corridor_max_iter", planner_config_->corridor_max_iter, planner_config_->corridor_max_iter);
  nh.param("corridor_incremental_limit", planner_config_->corridor_incremental_limit, planner_config_->corridor_incremental_limit);
  nh.param("weight_a", planner_config_->opti_w_a, planner_config_->opti_w_a);
  nh.param("weight_omega", planner_config_->opti_w_omega, planner_config_->opti_w_omega);
  nh.param("max_iter", planner_config_->opti_inner_iter_max, planner_config_->opti_inner_iter_max);
  nh.param("infeasible_penalty", planner_config_->opti_w_penalty0, planner_config_->opti_w_penalty0);
  nh.param("infeasible_tolerance", planner_config_->opti_varepsilon_tol, planner_config_->opti_varepsilon_tol);

  nh.param("front_hang_length", planner_config_->vehicle.front_hang_length, planner_config_->vehicle.front_hang_length);
  nh.param("wheel_base", planner_config_->vehicle.wheel_base, planner_config_->vehicle.wheel_base);
  nh.param("rear_hang_length", planner_config_->vehicle.rear_hang_length, planner_config_->vehicle.rear_hang_length);
  nh.param("width", planner_config_->vehicle.width, planner_config_->vehicle.width);

  nh.param("max_velocity", planner_config_->vehicle.max_velocity, planner_config_->vehicle.max_velocity);
  nh.param("min_velocity", planner_config_->vehicle.min_velocity, planner_config_->vehicle.min_velocity);
  nh.param("max_acceleration", planner_config_->vehicle.max_acceleration, planner_config_->vehicle.max_acceleration);
  nh.param("max_steering", planner_config_->vehicle.phi_max, planner_config_->vehicle.phi_max);
  nh.param("max_steering_rate", planner_config_->vehicle.omega_max, planner_config_->vehicle.omega_max);
  nh.param("n_disc", planner_config_->vehicle.n_disc, planner_config_->vehicle.n_disc);
}

// Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
void hmfpcLocalPlannerROS::initialize(std::string name,
                                     costmap_2d::Costmap2DROS *costmap_ros) {
  if (!initialized_) {
    costmap_ros_ = costmap_ros;
    name_ = name;
    initialized_ = true;
    planner_config_ = std::make_shared<heterogeneous_formation_controller::PlannerConfig>();
    planner_config_->vehicle.InitializeDiscs();

    env_ = std::make_shared<heterogeneous_formation_controller::Environment>(planner_config_);
    planner_ = std::make_shared<heterogeneous_formation_controller::hmfpcLocalPlanner>(planner_config_, env_);
    // ros::NodeHandle nh("~/" + name_);
    // visualization::Init(nh, config_.global_frame, "hmfpc_markers");
    odom_helper_.setOdomTopic(config_.odom_topic);
  ros::NodeHandle nh;
    ReadParameters(nh);
  visualization::Init(nh, "odom", "/hmfpc_test_vis");
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/hmfpc_test_path", 1, false);
  ros::Publisher path_pub_diff1 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_diff1", 1, false);
  ros::Publisher path_pub_diff2 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_diff2", 1, false);
  ros::Publisher path_pub_car1 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_car1", 1, false);
  FullStates solution, solution_car;
  FullStates solution_car_like1, solution_diff_drive1, solution_diff_drive2;
  double max_error1, coarse_path_time, solve_time_leader, solve_time_diff1, solve_time_diff2, solution_car_like;
  TrajectoryPoint start, goal;
  start.x = -15;
  start.y = -15;
  start.theta = M_PI / 2;
  goal.x = 15;
  goal.y = 15;
  goal.theta = M_PI / 2;

  goal_reached_ = false;
  env_->UpdateCostmapObstacles(costmap_ros_->getCostmap());
//   std::vector<double> new_vector;
//   new_vector.resize(2, 0.0);
//   for(int i = 0; i < env_->points().size(); i++) {
//     new_vector[0] = env_->points()[i].x();
//     new_vector[1] = env_->points()[i].y();
//     writeVectorToYAML(new_vector, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/costmap.yaml");
//   }
  iris::IRISProblem iris_problem(2);
  geometry_msgs::PoseStamped current_pose;
  start_state_.x = current_pose.pose.position.x;
  start_state_.y = current_pose.pose.position.y;
  start_state_.theta = math::NormalizeAngle(tf2::getYaw(current_pose.pose.orientation));

  geometry_msgs::PoseStamped current_vel;
  odom_helper_.getRobotVel(current_vel);
  start_state_.v = current_vel.pose.position.x;

  if(std::fabs(start_state_.v) > 1e-3) {
    double dtheta = tf2::getYaw(current_vel.pose.orientation);
    start_state_.phi = atan(dtheta / start_state_.v * planner_config_->vehicle.wheel_base);
  }

  if(solution_.states.size() > 1) {
    // assume control inputs reached desired values
    start_state_.a = solution_.states[1].a;
    start_state_.omega = solution_.states[1].omega;
  }
  start_state_.x = -60;
  start_state_.y = 36;
  start_state_.theta = 0;
  start_state_.v = 0;
  start_state_.a = 0;
  start_state_.phi = 0;
  start_state_.omega = 0;
  
  goal_state_.x = 35;
  goal_state_.y = 5;
  goal_state_.theta = 0;
  goal_state_.v = 0;
  goal_state_.a = 0;
  goal_state_.phi = 0;
  goal_state_.omega = 0;
  double solve_time_car = 0.0;
  bool show_cr = false;
  const std::vector<heterogeneous_formation_controller::visualization::Vector> corridor_sets;
  while (ros::ok()) {
  bool plan_result = planner_->Plan(solution_, start_state_, goal_state_, iris_problem, solution_, coarse_path_time, solve_time_leader, show_cr, corridor_sets);
  if (!planner_->Plan_car_like(solution_, 1.5 * 2.016733665, solution_car_like1, solve_time_car)) {
  }
  double infeasible = 0.0;
  if (!planner_->Plan_car_like_replan(solution_car_like1, solution_car, solution_car, 1, infeasible, solution_car_like)) {
  }
  auto traj_leader = fulltotraj(solution_);
  auto traj_fol_diff1 = generate_ref_traj_diff(traj_leader, 1.5 * hypot(-18-(-20), -21.0083668325-(-20)), atan2(21.0083668325-20, 20-18));

  if(!planner_->Plan_diff_drive(traj_fol_diff1, solution_diff_drive1, start, goal, solution_diff_drive1, 1, max_error1, solve_time_diff1)) {
  }
  DrawTrajectoryRvizRos(solution_, planner_config_, 0, path_pub);
  DrawTrajectoryRvizRos(solution_car, planner_config_, 1, path_pub_car1);
  DrawTrajectoryRvizRos(solution_diff_drive1, planner_config_, 2, path_pub_diff1);
    path_pub_ = nh.advertise<nav_msgs::Path>("local_path", 1);
  }
  }
}

// bool hmfpcLocalPlannerROS::makePlan(const geometry_msgs::PoseStamped& start, 
//           const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
//   if (!initialized_) {
//     ROS_ERROR("This planner has not been initialized");
//     return false;
//   }

//   geometry_msgs::PoseStamped goal_pose;
//   if(!base_local_planner::getGoalPose(*tf_, orig_global_plan, config_.global_frame, goal_pose)) {
//     ROS_ERROR("get goal pose failed");
//     return false;
//   }

//   goal_state_.x = goal_pose.pose.position.x;
//   goal_state_.y = goal_pose.pose.position.y;
//   goal_state_.theta = math::NormalizeAngle(tf2::getYaw(goal_pose.pose.orientation));
//   goal_reached_ = false;

//   return true;
// }

bool hmfpcLocalPlannerROS::makePlan(const geometry_msgs::PoseStamped& start_, 
          const geometry_msgs::PoseStamped& goal_, std::vector<geometry_msgs::PoseStamped>& plan) {
  FullStates solution, solution_car;
  FullStates solution_car_like1, solution_diff_drive1, solution_diff_drive2;
  double max_error1, coarse_path_time, solve_time_leader, solve_time_diff1, solve_time_diff2, solution_car_like;
  TrajectoryPoint start, goal;
  start.x = -15;
  start.y = -15;
  start.theta = M_PI / 2;
  goal.x = 15;
  goal.y = 15;
  goal.theta = M_PI / 2;
  if (!initialized_) {
    ROS_ERROR("This planner has not been initialized");
    return false;
  }

  goal_reached_ = false;
  env_->UpdateCostmapObstacles(costmap_ros_->getCostmap());
  iris::IRISProblem iris_problem(2);
  Eigen::MatrixXd obs = Eigen::MatrixXd::Zero(2, 4);
  obs << -33 + 1, -33 - 1, -33 - 1, -33 + 1,
        28.8 + 1, 28.8 + 1, 28.8 - 1, 28.8 - 1;
  obs << -33 + 1, -33 - 1, -33 - 1, -33 + 1,
        21 + 1, 21 + 1, 21 - 1, 21 - 1;
  obs << -20 + 1, -20 - 1, -20 - 1, -20 + 1,
        28.8 + 1, 28.8 + 1, 28.8 - 1, 28.8 - 1;
  obs << -20 + 1, -20 - 1, -20 - 1, -20 + 1,
        21 + 1, 21 + 1, 21 - 1, 21 - 1;
  obs << -43.7, -74.3, -73.7, -45.2,
        52.9, 52.7, 45.2, 45.5;
  obs << -74, -81.2, -74.3, -81.2,
        52.2, 52.5, 10, 10;
  obs << -74, -73.6, -51.3, -51,
        10, 16.1, 22.1, 10.7;
  obs << -49.7, -53.1, -52.3, -51,
        10, 10, 0, 0;
  obs << -49.7, -49.1, 75, 75,
        0, -2, -2, 0;
  obs << -42.6, -1.3, -1.6, -46.1,
        51.3, 51.1, 56, 56;
  obs << -0.65, -1.3, 24.2, 23.6,
        52.5, 8.75, 8, 51.2;
  obs << 39.5 + 1, 39.5 - 1, 39.5 - 1, 39.5 + 1,
        20 + 1, 20 + 1, 20 - 1, 20 - 1;
  obs << 72.5, 76.7, 76.7, 72.5,
        -1.5, -1.5, 50, 50;
  // for(int i = 0; i < env_->points().size(); i++) {
  //   obs << env_->points()[i].x() + 0.25, env_->points()[i].x() - 0.25, env_->points()[i].x() - 0.25, env_->points()[i].x() + 0.25,
  //          env_->points()[i].y() + 0.25, env_->points()[i].y() + 0.25, env_->points()[i].y() - 0.25, env_->points()[i].y() - 0.25;
  //   iris_problem.addObstacle(obs);
  // }
  iris_problem.addObstacle(obs);

  geometry_msgs::PoseStamped current_pose;
  if(!costmap_ros_->getRobotPose(current_pose)) {
    ROS_ERROR("failed to get robot pose");
    return false;
  }
  start_state_.x = current_pose.pose.position.x;
  start_state_.y = current_pose.pose.position.y;
  start_state_.theta = math::NormalizeAngle(tf2::getYaw(current_pose.pose.orientation));

  geometry_msgs::PoseStamped current_vel;
  odom_helper_.getRobotVel(current_vel);
  start_state_.v = current_vel.pose.position.x;

  if(std::fabs(start_state_.v) > 1e-3) {
    double dtheta = tf2::getYaw(current_vel.pose.orientation);
    start_state_.phi = atan(dtheta / start_state_.v * planner_config_->vehicle.wheel_base);
  }

  if(solution_.states.size() > 1) {
    // assume control inputs reached desired values
    start_state_.a = solution_.states[1].a;
    start_state_.omega = solution_.states[1].omega;
  }
  start_state_.x = -60;
  start_state_.y = 36;
  start_state_.theta = 0;
  start_state_.v = 0;
  start_state_.a = 0;
  start_state_.phi = 0;
  start_state_.omega = 0;
  
  goal_state_.x = goal_.pose.position.x;
  goal_state_.y = goal_.pose.position.y;
  goal_state_.theta = 0;
  goal_state_.v = 0;
  goal_state_.a = 0;
  goal_state_.phi = 0;
  goal_state_.omega = 0;

  if(std::fabs(start_state_.x - goal_state_.x) < config_.goal_xy_tolerance && std::fabs(start_state_.y - goal_state_.y) < config_.goal_xy_tolerance && std::fabs(math::AngleDiff(start_state_.theta, goal_state_.theta)) < config_.goal_yaw_tolerance) {
    goal_reached_ = true;
    solution_ = decltype(solution_)();
    return true;
  }
  bool show_cr = false;
  const std::vector<std::vector<double>> corridor_sets;
  bool plan_result = planner_->Plan(solution_, start_state_, goal_state_, iris_problem, solution_, coarse_path_time, solve_time_leader, show_cr, corridor_sets);
  if(!plan_result) {
    ROS_ERROR("local plan failed");
    return false;
  }
  double solve_time_car = 0.0;
  if (!planner_->Plan_car_like(solution_, 2.016733665, solution_car_like1, solve_time_car)) {
  }
  double infeasible = 0.0;
  if (!planner_->Plan_car_like_replan(solution_car_like1, solution_car, solution_car, 1, infeasible, solution_car_like)) {
  }
  auto traj_leader = fulltotraj(solution_);
  auto traj_fol_diff1 = generate_ref_traj_diff(traj_leader, hypot(-18-(-20), -21.0083668325-(-20)), atan2(21.0083668325-20, 20-18));

  if(!planner_->Plan_diff_drive(traj_fol_diff1, solution_diff_drive1, start, goal, solution_diff_drive1, 1, max_error1, solve_time_diff1)) {
  }
  ros::NodeHandle nh;
  visualization::Init(nh, "odom", "/hmfpc_test_vis");
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/hmfpc_test_path", 1, false);
  ros::Publisher path_pub_diff1 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_diff1", 1, false);
  ros::Publisher path_pub_diff2 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_diff2", 1, false);
  ros::Publisher path_pub_car1 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_car1", 1, false);
  DrawTrajectoryRvizRos(solution_, planner_config_, 0, path_pub);
  DrawTrajectoryRvizRos(solution_car, planner_config_, 1, path_pub_car1);
  DrawTrajectoryRvizRos(solution_diff_drive1, planner_config_, 2, path_pub_diff1);
  nav_msgs::Path msg;
  msg.header.frame_id = config_.global_frame;
  msg.header.stamp = ros::Time::now();
  for(size_t i = 0; i < solution_.states.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = solution_.states[i].x;
    pose.pose.position.y = solution_.states[i].y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(solution_.states[i].theta);
    msg.poses.push_back(pose);
  }

  path_pub_.publish(msg);
  return true;
}

}

#ifndef LOCAL_PLAN_FOLLOWER_H_
#define LOCAL_PLAN_FOLLOWER_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
// #include <base_local_planner/latched_stop_rotate_controller.h>
// #include <base_local_planner/odometry_helper_ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace local_plan_follower
{

class LocalPlanFollower : public nav_core::BaseLocalPlanner
{
public:
    LocalPlanFollower();

    LocalPlanFollower(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

    ~LocalPlanFollower();

    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    bool isGoalReached();

private:
    void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

    ros::Publisher local_path_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;
    bool initialized_;
    std::string odom_topic_;
    geometry_msgs::PoseStamped current_pose_;
    // base_local_planner::LocalPlannerUtil planner_util_;
    // base_local_planner::OdometryHelperRos odom_helper_;
    // base_local_planner::LatchedStopRotateController latched_stop_rotate_controller_;
};
};  // namespace local_plan_follower

#endif
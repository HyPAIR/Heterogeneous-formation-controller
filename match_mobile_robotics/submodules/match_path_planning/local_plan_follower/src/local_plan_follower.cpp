#include <local_plan_follower/local_plan_follower.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(local_plan_follower::LocalPlanFollower, nav_core::BaseLocalPlanner)

namespace local_plan_follower
{

LocalPlanFollower::LocalPlanFollower() : costmap_ros_(NULL), tf_(NULL), initialized_(false)  //, odom_helper_("odom")
{
}

LocalPlanFollower::LocalPlanFollower(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false)  //, odom_helper_("odom")
{
    initialize(name, tf, costmap_ros);
}

LocalPlanFollower::~LocalPlanFollower(){};

void LocalPlanFollower::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    if (!initialized_)
    {
        std::shared_ptr<tf2_ros::Buffer> tf_ptr(tf);
        tf_ = tf_ptr;
        std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ptr(costmap_ros);
        costmap_ros_ = costmap_ptr;
        initialized_ = true;
    }
}

bool LocalPlanFollower::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
    if (!initialized_)
    {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
    }
    ROS_INFO("Always return true for setPlan!");

    return true;
}

bool LocalPlanFollower::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    if (!initialized_)
    {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
    }
    ROS_INFO("Always return zero velocity!");

    return true;
}

bool LocalPlanFollower::isGoalReached()
{
    if (!initialized_)
    {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
    }
    ROS_INFO("Always return true!");

    return true;
}
}  // namespace local_plan_follower
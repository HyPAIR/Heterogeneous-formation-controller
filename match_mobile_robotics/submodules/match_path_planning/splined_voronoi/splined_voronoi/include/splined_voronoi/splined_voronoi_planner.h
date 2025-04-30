#pragma once

#include <chrono>
#include <angles/angles.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float64MultiArray.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <navfn/MakeNavPlan.h>
#include <dynamic_reconfigure/server.h>
#include <splined_voronoi/voronoi_generation.h>
#include <splined_voronoi/path_planning.h>
#include <splined_voronoi/path_smoothing.h>
#include <splined_voronoi/SplinedVoronoiPlannerConfig.h>
#include <splined_voronoi/MakePlanWithStats.h>

namespace splined_voronoi
{

class SplinedVoronoiPlanner : public nav_core::BaseGlobalPlanner
{
public:
    /**
     * @brief Construct a new SplinedVoronoiPlanner object
     *
     */
    SplinedVoronoiPlanner();

    /**
     * @brief Construct a new Splined Voronoi Planner object
     *
     * @param name The name of this planner
     * @param costmap_ros Costmap for obstacle avoidance
     */
    SplinedVoronoiPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief Destructor the Splined Voronoi Planner object.
     *
     * Deletes all existing pointers.
     */
    ~SplinedVoronoiPlanner();

    /**
     * @brief Initialization function for the SplinedVoronoiPlanner.
     *
     * overrides class from interface nav_core::BaseGlobalPlanner
     *
     * @param name The name of this planner
     * @param costmap_ros Costmap for obstacle avoidance
     */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief Compute Plan from given start and goal pose
     *
     * @param start Start Pose in map frame
     * @param goal Goal Pose in map frame
     * @param plan output plan
     * @return true when plan is found;
     * @return false otherwise
     */
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

    /**
     * @brief ROS-Service interface to compute a plan
     *
     * @param req Request as defined in navfn/MakeNavPlan.srv
     * @param res Response as defined in navfn/MakeNavPlan.srv
     * @return true if service call was successful;
     * @return false otherwise
     */
    bool makePlanService(navfn::MakeNavPlan::Request& req, navfn::MakeNavPlan::Response& res);

    /**
     * @brief ROS-Service interface with more stats of planning
     *
     * @param req Request as defined in custom srv
     * @param res Response as defined in custom srv
     * @return true if service call was successful;
     * @return false otherwise
     */
    bool makePlanWithStatsService(splined_voronoi::MakePlanWithStats::Request& req, splined_voronoi::MakePlanWithStats::Response& res);

    /**
     * @brief Requests the planner to cancel, e.g. if it takes too much time.
     * @remark New on MBF API
     * @return True if a cancel has been successfully requested, false if not implemented.
     */
    bool cancel();

private:
    bool initialized_;
    bool optimize_lengths_;
    bool perform_splining_;
    bool fast_mode_;
    std::shared_ptr<costmap_2d::Costmap2D> costmap_;
    std::string costmap_global_frame_;
    cv::Point2d map_origin_;
    int costmap_size_x_;
    int costmap_size_y_;
    double costmap_resolution_;  // m / px
    double free_cell_threshold_;
    double angle_threshold_;
    double min_distance_control_points_m_;
    double plan_resolution_;
    double max_curvature_;
    double curvature_safety_margin_;
    double free_space_factor_;
    double max_optimization_time_;
    std::vector<std::pair<double, double>> robot_formation_;
    boost::mutex mutex_;
    ros::ServiceServer make_plan_service_;
    ros::ServiceServer make_plan_with_stats_service_;
    ros::Publisher plan_pub_;
    ros::Publisher original_plan_pub_;
    ros::Publisher sparse_plan_pub_;
    ros::Publisher spline_plan_initial_pub_;
    ros::Publisher optimized_plan_pub_;
    ros::Publisher optimized_lengths_pub_;
    ros::Publisher voronoi_map_pub_;

    geometry_msgs::PoseStamped start_;
    geometry_msgs::PoseStamped goal_;
    cv::Point2i start_map_;
    cv::Point2i goal_map_;
    cv::Point2d start_world_;
    cv::Point2d goal_world_;

    cv::Mat voronoi_img_;
    cv::Mat obstacle_img_;
    cv::Mat obstacle_img_wo_unknowns_;

    // output for service response
    std::vector<geometry_msgs::PoseStamped> astar_path_;
    std::vector<geometry_msgs::PoseStamped> sparse_path_;
    std::vector<geometry_msgs::PoseStamped> optimized_sparse_path_;
    std::vector<double> spline_tangent_lengths_;
    double time_taken_;

    dynamic_reconfigure::Server<splined_voronoi::SplinedVoronoiPlannerConfig>* dsrv_;

    /**
     * @brief Callback for dynamic reconfigure
     *
     * @param config current configuration
     * @param level level of change
     */
    void reconfigureCB(splined_voronoi::SplinedVoronoiPlannerConfig &config, uint32_t level);


    /** @brief publishes Plan for visualization with rviz; taken from https://github.com/frontw/voronoi_planner
     *
     * @param path the path to be published
     * @param pub publisher to be used
     */
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub);
};
};  // namespace splined_voronoi

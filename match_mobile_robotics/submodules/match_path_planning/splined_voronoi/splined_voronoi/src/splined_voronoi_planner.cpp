#include <splined_voronoi/splined_voronoi_planner.h>
#include <pluginlib/class_list_macros.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(splined_voronoi::SplinedVoronoiPlanner, nav_core::BaseGlobalPlanner)

namespace splined_voronoi
{

SplinedVoronoiPlanner::SplinedVoronoiPlanner()
{
    ROS_INFO("SplinedVoronoiPlanner empty Constructor got called!");
}

SplinedVoronoiPlanner::SplinedVoronoiPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    ROS_INFO("SplinedVoronoiPlanner Constructor got called!");
    initialize(name, costmap_ros);
}

SplinedVoronoiPlanner::~SplinedVoronoiPlanner()
{
    ROS_INFO("SplinedVoronoiPlanner Deconstructor called");
    if (this->dsrv_)
    {
        delete this->dsrv_;
    }
}

void SplinedVoronoiPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    if (!initialized_)
    {
        ROS_INFO("SplinedVoronoiPlanner: Init got called!");
        // ROS_INFO_STREAM("Planner Name: " << name);
        ros::NodeHandle private_nh("~/" + name);
        // costmap_layered = costmap_ros->getLayeredCostmap()
        costmap_ = std::shared_ptr<costmap_2d::Costmap2D>(costmap_ros->getCostmap());
        costmap_global_frame_ = costmap_ros->getGlobalFrameID();
        costmap_size_x_ = costmap_->getSizeInCellsX();
        costmap_size_y_ = costmap_->getSizeInCellsY();
        costmap_resolution_ = costmap_->getResolution();
        this->map_origin_ = cv::Point2d(this->costmap_->getOriginX(), this->costmap_->getOriginY());

        ROS_INFO_STREAM("Costmap Size: " << costmap_size_x_ << ", " << costmap_size_y_);
        ROS_INFO_STREAM("Costmap Origin: " << map_origin_.x << ", " << map_origin_.y);
        ROS_INFO_STREAM("Costmap Resolution: " << costmap_resolution_);

        // get params
        private_nh.param("free_cell_threshold", free_cell_threshold_, 0.0);
        private_nh.param("angle_threshold", angle_threshold_, M_PI_4 / 2);
        private_nh.param("min_distance_control_points", min_distance_control_points_m_, 0.0);
        private_nh.param("plan_resolution", plan_resolution_, 50.0);
        private_nh.param("max_curvature", this->max_curvature_, -1.0);
        private_nh.param("curvature_safety_margin", curvature_safety_margin_, 0.05);
        private_nh.param("max_optimization_time", max_optimization_time_, 4.0);
        private_nh.param("free_space_factor", this->free_space_factor_, 4.0);
        private_nh.param("optimize_lengths", optimize_lengths_, false);
        private_nh.param("perform_splining", this->perform_splining_, true);
        private_nh.param("fast_mode", this->fast_mode_, false);
        ROS_INFO_STREAM("Load param free_cell_threshold: " << free_cell_threshold_);
        ROS_INFO_STREAM("Load param angle_threshold: " << angle_threshold_);
        ROS_INFO_STREAM("Load param min_distance_control_points: " << min_distance_control_points_m_);
        ROS_INFO_STREAM("Load param max_curvature: " << max_curvature_);
        ROS_INFO_STREAM("Load param curvature_safety_margin: " << curvature_safety_margin_);

        bool get_curvature_from_formation = this->max_curvature_ == -1.0;
        std::string get_curve_msg = get_curvature_from_formation ? "True" : "False";
        ROS_INFO_STREAM("Calculating curvature from formation: " << get_curve_msg);
        std::vector<std::string> robot_names;
        XmlRpc::XmlRpcValue formation_robots;
        private_nh.param("formation_config/robot_names", formation_robots, formation_robots);
        double max_curvature_from_formation = std::numeric_limits<double>::infinity();
        for (int i = 0; i < formation_robots.size(); i++)
        {
            std::string robot_namespace = formation_robots[i];
            robot_names.push_back(robot_namespace);
            double rel_x_offset = 0.0;
            double rel_y_offset = 0.0;
            double rel_yaw_offset = 0.0;
            private_nh.param("formation_config" + robot_namespace + "/rel_x_offset", rel_x_offset, rel_x_offset);
            private_nh.param("formation_config" + robot_namespace + "/rel_y_offset", rel_y_offset, rel_y_offset);
            private_nh.param("formation_config" + robot_namespace + "/rel_yaw_offset", rel_yaw_offset, rel_yaw_offset);
            ROS_INFO_STREAM("Robot namespace " << robot_namespace << " : Offset (" << rel_x_offset << ", " << rel_y_offset << ")");
            this->robot_formation_.push_back({rel_x_offset, rel_y_offset});
            if (get_curvature_from_formation)
            {
                double min_radius = sqrt(pow(rel_x_offset, 2) + pow(rel_y_offset, 2));
                if (rel_x_offset != 0.0)
                {
                    min_radius *= cos(atan(rel_y_offset / rel_x_offset));
                }
                ROS_INFO_STREAM("Got min radius: " << min_radius);
                if (1.0 / min_radius < max_curvature_from_formation)
                {
                    max_curvature_from_formation = 1.0 / min_radius;
                    this->max_curvature_ = max_curvature_from_formation;
                }
            }
        }
        ROS_INFO_STREAM("max_curvature from formation: " << max_curvature_);
        this->max_curvature_ *= 1.0 - this->curvature_safety_margin_;
        ROS_INFO_STREAM("max_curvature after applying safety margin: " << max_curvature_);

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("/voronoi_planner/formation_plan", 1);
        original_plan_pub_ = private_nh.advertise<nav_msgs::Path>("original_plan", 1);
        sparse_plan_pub_ = private_nh.advertise<nav_msgs::Path>("sparse_plan", 1);
        spline_plan_initial_pub_ = private_nh.advertise<nav_msgs::Path>("spline_plan_initial", 1);
        optimized_plan_pub_ = private_nh.advertise<nav_msgs::Path>("optimized_plan", 1);
        optimized_lengths_pub_ = private_nh.advertise<std_msgs::Float64MultiArray>("optimized_lengths", 1);
        voronoi_map_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("voronoi_map", 1);

        for (int idx = 1; idx <= this->robot_formation_.size(); idx++)
        {
            std::string topic_name = "/voronoi_planner/formation_plan/robot" + std::to_string(idx);
        }

        this->make_plan_service_ = private_nh.advertiseService("make_plan", &SplinedVoronoiPlanner::makePlanService, this);
        this->make_plan_with_stats_service_ = private_nh.advertiseService("make_plan_with_stats", &SplinedVoronoiPlanner::makePlanWithStatsService, this);

        this->dsrv_ = new dynamic_reconfigure::Server<splined_voronoi::SplinedVoronoiPlannerConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<splined_voronoi::SplinedVoronoiPlannerConfig>::CallbackType cb = boost::bind(&SplinedVoronoiPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        initialized_ = true;
    }
    else
    {
        ROS_ERROR("SplinedVoronoiPlanner already initialized, aborting init!");
    }
}

void SplinedVoronoiPlanner::reconfigureCB(splined_voronoi::SplinedVoronoiPlannerConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request");
    this->free_cell_threshold_ = config.free_cell_threshold;
    double max_curvature = config.max_curvature;
    double safety_margin = config.curvature_safety_margin;
    this->max_curvature_ = max_curvature * (1.0 - safety_margin);
    this->angle_threshold_ = config.angle_threshold * M_PI / 180.0;
    this->optimize_lengths_ = config.optimize_lengths;
    this->min_distance_control_points_m_ = config.min_distance_control_points;
    this->max_optimization_time_ = config.max_optimization_time;
    this->free_space_factor_ = config.free_space_factor;
    this->perform_splining_ = config.perform_splining;
    this->fast_mode_ = config.fast_mode;
    ROS_INFO_STREAM("Max curvature is now: " << this->max_curvature_);
}

bool SplinedVoronoiPlanner::cancel()
{
    ROS_ERROR("SplinedVoronoiPlanner CANCEL");
    // Returns false if not implemented. Returns true if implemented and cancel has been successful.
    return false;
}


bool SplinedVoronoiPlanner::makePlanWithStatsService(splined_voronoi::MakePlanWithStats::Request& req, splined_voronoi::MakePlanWithStats::Response& res)
{
    std::vector<geometry_msgs::PoseStamped> path;

    req.start.header.frame_id = "map";
    req.goal.header.frame_id = "map";
    bool success = makePlan(req.start, req.goal, path);
    if (success)
    {
        res.plan_found = 0;
    }
    else
    {
        res.plan_found = -1;
    }
    res.time_taken = this->time_taken_;
    res.astar_path = this->astar_path_;
    res.sparse_path = this->sparse_path_;
    res.optimized_sparse_path = this->optimized_sparse_path_;
    res.spline_tangent_lengths = this->spline_tangent_lengths_;
    res.path = path;
    return true;
}

bool SplinedVoronoiPlanner::makePlanService(navfn::MakeNavPlan::Request& req, navfn::MakeNavPlan::Response& res)
{
    std::vector<geometry_msgs::PoseStamped> path;

    req.start.header.frame_id = "map";
    req.goal.header.frame_id = "map";
    bool success = makePlan(req.start, req.goal, path);
    res.plan_found = success;
    if (success)
    {
        res.path = path;
    }

    return true;
}

bool SplinedVoronoiPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                    std::vector<geometry_msgs::PoseStamped>& plan)
{
    boost::mutex::scoped_lock lock(mutex_);
    ROS_INFO("SplinedVoronoiPlanner makePlan got called!");
    if (!initialized_)
    {
        ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    this->start_ = start;
    this->goal_ = goal;

    // clear the plan, just in case
    plan.clear();

    this->astar_path_.clear();
    this->sparse_path_.clear();
    this->optimized_sparse_path_.clear();
    this->spline_tangent_lengths_.clear();

    std::string global_frame = costmap_global_frame_;

    // require the goal to be in our global frame
    if (goal.header.frame_id != global_frame)
    {
        ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
                  global_frame.c_str(), goal.header.frame_id.c_str());
        return false;
    }

    if (start.header.frame_id != global_frame)
    {
        ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
                  global_frame.c_str(), start.header.frame_id.c_str());
        return false;
    }

    this->start_world_ = cv::Point2d(start.pose.position.x, start.pose.position.y);

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;

    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x_i, start_y_i))
    {
        ROS_ERROR(
            "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot "
            "has been properly localized?");
        return false;
    }
    this->start_map_.x = start_x_i;
    this->start_map_.y = start_y_i;

    this->goal_world_ = cv::Point2d(goal.pose.position.x, goal.pose.position.y);

    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x_i, goal_y_i))
    {
        ROS_ERROR("The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    this->goal_map_.x = goal_x_i;
    this->goal_map_.y = goal_y_i;

    double angle_start = 2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
    double angle_goal = 2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);

    ROS_WARN_STREAM("Planning from " << this->start_world_ << ", " << angle_start << " to " << this->goal_world_ << ", " << angle_goal);

    std::chrono::steady_clock::time_point coordinate_transform_time = std::chrono::steady_clock::now();
    double coord_trans_duration = (std::chrono::duration_cast<std::chrono::microseconds>(coordinate_transform_time - begin).count()) / 1000000.0;
    // ROS_INFO_STREAM("SplinedVoronoiPlanner: Time for start and goal coordinate transforms (s): " << coord_trans_duration);

    this->costmap_size_x_ = this->costmap_->getSizeInCellsX();
    this->costmap_size_y_ = this->costmap_->getSizeInCellsY();
    cv::Mat costmap_img_orig = cv::Mat::zeros(this->costmap_size_x_, this->costmap_size_y_, CV_8UC1);
    cv::Mat costmap_img_wo_unknowns = cv::Mat::zeros(this->costmap_size_x_, this->costmap_size_y_, CV_8UC1);
    int obstacle_count = 0;
    int unknown_counter = 0;
    for (int i = 0; i < this->costmap_size_x_; i++)
    {
        for (int j = 0; j < this->costmap_size_y_; j++)
        {
            double cell_cost = this->costmap_->getCost(i, j);
            if (cell_cost != costmap_2d::NO_INFORMATION && cell_cost > free_cell_threshold_)
            {
                costmap_img_wo_unknowns.at<uchar>(i, j, 0) = 255;
            }
            if (cell_cost > free_cell_threshold_)
            {
                costmap_img_orig.at<uchar>(i, j, 0) = 255;
                obstacle_count++;
            }
        }
    }
    // ROS_WARN_STREAM("Unknowns: " << unknown_counter << ", value: " << costmap_2d::NO_INFORMATION);
    // enlargen obstacles by formation radius
    cv::Mat costmap_dist_img;
    cv::distanceTransform(~costmap_img_orig, costmap_dist_img, cv::DIST_L2, 3, CV_8UC1);
    cv::Mat costmap_img;
    cv::threshold(~costmap_img_orig, costmap_img, 1 / this->costmap_resolution_ / this->max_curvature_, 255, cv::THRESH_BINARY_INV);
    costmap_img.convertTo(costmap_img, CV_8UC1);

    this->obstacle_img_ = costmap_img;
    this->obstacle_img_wo_unknowns_ = costmap_img_wo_unknowns;

    // check if start or goal is in collision
    if (this->obstacle_img_.at<uchar>(this->start_map_.x, this->start_map_.y) == 255 || this->obstacle_img_.at<uchar>(this->goal_map_.x, this->goal_map_.y) == 255)
    {
        ROS_ERROR("Start or goal is inside obstacle. Planning will always fail. Aborting..");
        return false;
    }

    // check if start and goal are within the same connected component
    cv::Mat label_img;
    int num_labels = cv::connectedComponents(~this->obstacle_img_wo_unknowns_, label_img, 8);
    if (label_img.at<int>(this->start_map_.x, this->start_map_.y) != label_img.at<int>(this->goal_map_.x, this->goal_map_.y))
    {
        ROS_ERROR("Start and goal are not in the same connected component; Planning will always fail. Aborting..");
        return false;
    }

    std::chrono::steady_clock::time_point costmap_as_img_time = std::chrono::steady_clock::now();
    double costmap_as_img_duration = (std::chrono::duration_cast<std::chrono::microseconds>(costmap_as_img_time - coordinate_transform_time).count()) / 1000000.0;
    // ROS_INFO_STREAM("SplinedVoronoiPlanner: Time taken for putting costmap in image (s): " << costmap_as_img_duration);

    // generate voronoi representation of map only when there are obstacles
    if (obstacle_count == 0)
    {
        ROS_WARN("No obstacles found in costmap, skipping voronoi generation.");
        this->voronoi_img_ = cv::Mat(this->costmap_size_x_, this->costmap_size_y_, CV_8UC1, cv::Scalar(255));
    }
    // check if voronoi img already exists
    else if (fast_mode_ && this->voronoi_img_.rows == this->costmap_size_x_ && this->voronoi_img_.cols == this->costmap_size_y_)
    {
        ROS_WARN("Voronoi img already exists, skipping voronoi generation.");
    }
    else
    {
        bool voronoi_creation_success = voronoi_generation::create_boost_voronoi(costmap_img, this->voronoi_img_);
    }

    std::chrono::steady_clock::time_point voronoi_generation_time = std::chrono::steady_clock::now();
    double voronoi_generation_duration = (std::chrono::duration_cast<std::chrono::microseconds>(voronoi_generation_time - costmap_as_img_time).count()) / 1000000.0;
    ROS_INFO_STREAM("SplinedVoronoiPlanner: Time for generating voronoi img (s): " << voronoi_generation_duration);

    // get areas with large freespaces and overly voronoi diagram with it
    cv::Mat costmap_large_spaces;
    cv::threshold(costmap_dist_img, costmap_large_spaces, this->free_space_factor_  / this->costmap_resolution_, 255, cv::THRESH_BINARY);
    costmap_large_spaces.convertTo(costmap_large_spaces, CV_8UC1);
    cv::bitwise_or(this->voronoi_img_, costmap_large_spaces, this->voronoi_img_);

    // create Occupancy Grid which has voronoi data and obstacles for visualization
    nav_msgs::OccupancyGrid voronoi_map;
    voronoi_map.header.frame_id = this->costmap_global_frame_;
    voronoi_map.header.stamp = ros::Time::now();
    voronoi_map.info.map_load_time = ros::Time::now();
    voronoi_map.info.height = this->costmap_size_y_;
    voronoi_map.info.width = this->costmap_size_x_;
    voronoi_map.info.resolution = this->costmap_resolution_;
    voronoi_map.info.origin.position.x = this->costmap_->getOriginX();
    voronoi_map.info.origin.position.y = this->costmap_->getOriginY();
    voronoi_map.data.assign(voronoi_map.info.width * voronoi_map.info.height, 100);
    for (int i = 0; i < costmap_size_x_; i++)
    {
        for (int j = 0; j < costmap_size_y_; j++)
        {
            int occupancy_index = j * costmap_size_x_ + i;
            if (voronoi_img_.at<uchar>(i, j) == 255)
            {
                voronoi_map.data[occupancy_index] = 0;
            }
            else if (voronoi_img_.at<uchar>(i, j) > 0)
            {
                voronoi_map.data[occupancy_index] = 50;
            }
        }
    }
    voronoi_map_pub_.publish(voronoi_map);
    std::chrono::steady_clock::time_point voronoi_map_time = std::chrono::steady_clock::now();
    double voronoi_map_duration = (std::chrono::duration_cast<std::chrono::milliseconds>(voronoi_map_time - voronoi_generation_time).count()) / 1000.0;
    ROS_INFO_STREAM("SplinedVoronoiPlanner: Time for generating voronoi map (s): " << voronoi_map_duration);

    // get path on voronoi image
    std::vector<cv::Point2i> path;
    bool path_found = path_planning::findCompletePath(this->obstacle_img_, this->voronoi_img_, path, this->start_map_, this->goal_map_);
    std::chrono::steady_clock::time_point path_finding_time = std::chrono::steady_clock::now();
    double path_finding_duration = (std::chrono::duration_cast<std::chrono::milliseconds>(path_finding_time - voronoi_map_time).count()) / 1000.0;
    ROS_INFO_STREAM("SplinedVoronoiPlanner: Time taken for finding path (s): " << path_finding_duration);

    if (!path_found && path.empty())
    {
        ROS_ERROR("No valid path found, returning empty plan");
        return false;
    }
    else if (!path_found)
    {
        ROS_WARN("Direct Connection between Start and Goal");
    }

    // insert start and goal position to have accuracy beyond grid resolution
    if (path.empty())
    {
        // for an empty path return a plan from start to goal, otherwise dont include start to begin plan on voronoi
        path.insert(path.begin(), this->start_map_);
    }

    // create unreduced plan for debug output
    std::vector<geometry_msgs::PoseStamped> unreduced_plan;
    path_planning::createPlanFromPath(path, unreduced_plan, this->costmap_, this->costmap_global_frame_);
    this->astar_path_ = unreduced_plan;
    this->publishPlan(unreduced_plan, this->original_plan_pub_);

    if(!perform_splining_)
    {
        // return unsplined path
        for (auto pose : unreduced_plan)
        {
            plan.push_back(pose);
        }
        // publish plan for visualization
        this->publishPlan(plan, this->plan_pub_);
        return true;
    }

    // select waypoints from path
    double min_distance_control_points_px = this->min_distance_control_points_m_ / this->costmap_resolution_;
    std::vector<cv::Point2i> sparse_path;
    bool sparse_success = path_planning::sparsify_path(path, sparse_path, this->angle_threshold_, min_distance_control_points_px);

    // create sparse plan for debug output
    std::vector<geometry_msgs::PoseStamped> sparse_plan;
    path_planning::createPlanFromPath(sparse_path, sparse_plan, this->costmap_, this->costmap_global_frame_);
    this->sparse_path_ = sparse_plan;
    this->publishPlan(sparse_plan, this->sparse_plan_pub_);


    // convert path from Map to World
    std::vector<cv::Point2d> sparse_path_world;
    path_planning::pathMapToWorld(sparse_path, sparse_path_world, this->costmap_);

    std::chrono::steady_clock::time_point plan_creating_time = std::chrono::steady_clock::now();
    double plan_creating_duration = (std::chrono::duration_cast<std::chrono::microseconds>(plan_creating_time - path_finding_time).count()) / 1000000.0;
    // ROS_INFO_STREAM("SplinedVoronoiPlanner: Time taken to create plan from path (s): " << plan_creating_duration);

    std::vector<double> tangent_lengths;
    std::vector<cv::Point2d> spline_samples_initial;
    std::vector<geometry_msgs::PoseStamped> spline_plan_initial;
    path_smoothing::getSplinePathSamples(sparse_path_world, tangent_lengths, spline_samples_initial, 0.02, 1);
    path_planning::createPlanFromPath(spline_samples_initial, spline_plan_initial, this->costmap_global_frame_);
    this->publishPlan(spline_plan_initial, this->spline_plan_initial_pub_);

    std::chrono::steady_clock::time_point splining_time = std::chrono::steady_clock::now();
    double splining_duration = (std::chrono::duration_cast<std::chrono::microseconds>(splining_time - plan_creating_time).count()) / 1000000.0;
    ROS_INFO_STREAM("SplinedVoronoiPlanner: Time taken to create spline samples (s): " << splining_duration);

    // build spline from waypoints and optimize to fulfil curvature limit
    std::vector<cv::Point2d> continuous_path;
    std::vector<cv::Point2d> optimized_sparse_path;
    std::vector<double> optimized_lengths;
    bool splining_success = path_smoothing::buildOptimizedContinuousPath(sparse_path_world, continuous_path, optimized_sparse_path, optimized_lengths, *(this->costmap_), this->obstacle_img_wo_unknowns_, this->max_curvature_, this->plan_resolution_, this->optimize_lengths_, this->max_optimization_time_);
    std::string splining_status_msg = splining_success ? "Success" : "Failure";
    ROS_INFO_STREAM("Optimization Status: " << splining_status_msg);
    std_msgs::Float64MultiArray optimized_lengths_msg;
    optimized_lengths_msg.data = optimized_lengths;
    optimized_lengths_pub_.publish(optimized_lengths_msg);

    // create plan from optimized sparse path for visualization
    std::vector<geometry_msgs::PoseStamped> optimized_sparse_plan;
    path_planning::createPlanFromPath(optimized_sparse_path, optimized_sparse_plan, this->costmap_global_frame_);
    this->optimized_sparse_path_ = optimized_sparse_plan;
    this->spline_tangent_lengths_ = optimized_lengths;
    this->publishPlan(optimized_sparse_plan, this->optimized_plan_pub_);

    if (!splining_success)
    {
        return false;
    }

    std::vector<geometry_msgs::PoseStamped> splined_plan;
    path_planning::createPlanFromPath(continuous_path, splined_plan, this->costmap_global_frame_);
    // std::chrono::steady_clock::time_point splining_time = std::chrono::steady_clock::now();
    // double splining_duration = (std::chrono::duration_cast<std::chrono::milliseconds>(splining_time - plan_creating_time).count()) / 1000.0;
    // ROS_INFO_STREAM("SplinedVoronoiPlanner: Time taken for building splined plan (s): " << splining_duration);

    if (!path_planning::isPlanFree(costmap_, free_cell_threshold_, splined_plan))
    {
        ROS_ERROR("Plan is not free!");
        return false;
    }
    std::chrono::steady_clock::time_point free_check_time = std::chrono::steady_clock::now();
    ROS_INFO_STREAM(
        "SplinedVoronoiPlanner: Time taken for checking if final plan is free (s): "
        << (std::chrono::duration_cast<std::chrono::microseconds>(free_check_time - splining_time).count()) /
               1000000.0);
    
    // fill resulting plan
    for (auto pose : splined_plan)
    {
        plan.push_back(pose);
    }
    if (plan.empty())
    {
        ROS_ERROR("Got empty plan from spline interpolation");
        return false;
    }

    // publish plan for visualization
    this->publishPlan(plan, this->plan_pub_);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    double total_time_taken = (std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) / 1000000.0;
    ROS_INFO_STREAM("SplinedVoronoiPlanner: Total Time taken (s): " << total_time_taken);

    this->time_taken_ = total_time_taken;
    return true;
}


void SplinedVoronoiPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub)
{
    if (!initialized_)
    {
        ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    // create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if (!path.empty())
    {
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = ros::Time::now();
    }
    else
    {
        ROS_WARN_STREAM("Got empty plan for publishing");
        return;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++)
    {
        gui_path.poses[i] = path[i];
    }

    pub.publish(gui_path);
}

};  // namespace splined_voronoi
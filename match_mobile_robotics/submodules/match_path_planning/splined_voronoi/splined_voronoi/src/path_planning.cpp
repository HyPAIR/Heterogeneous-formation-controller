#include <splined_voronoi/path_planning.h>

namespace path_planning
{


std::vector<cv::Point2i> create_deltas(bool use_eight_neighbors)
{
    std::vector<cv::Point2i> deltas;
    deltas.push_back(cv::Point2i(-1, 0));  // go up
    deltas.push_back(cv::Point2i(0, -1));  // go left
    deltas.push_back(cv::Point2i(1, 0));   // go down
    deltas.push_back(cv::Point2i(0, 1));   // go right
    // use only 4 neighborhood to get 90 degree curves only, which are better for later interpolation
    if (use_eight_neighbors)
    {
        deltas.push_back(cv::Point2i(-1, -1));  // up and left
        deltas.push_back(cv::Point2i(-1, 1));   // up and right
        deltas.push_back(cv::Point2i(1, -1));   // down and left
        deltas.push_back(cv::Point2i(1, 1));    // down and right
    }
    return deltas;
}

int findDijkstraPathToVoronoi(const cv::Mat& obstacle_map, const cv::Mat& voronoi_map,
                               cv::Point2i& nearest_point_on_voronoi, cv::Point2i start, cv::Point2i goal,
                               bool use_eight_neighbors)
{
    std::vector<cv::Point2i> deltas = create_deltas(use_eight_neighbors);
    cv::Mat visited(voronoi_map.size(), CV_8UC1, cv::Scalar(0));
    visited.at<uchar>(start.x, start.y) = 255;
    float g = 0;
    int movement_cost = 1;  // doesnt matter in which direction or angle, one cell movement costs 1

    // vector of open cells (for possible expansion) nodes
    std::multiset<AStarCell, std::less<AStarCell>> array_open_cell_list;
    array_open_cell_list.insert({ start, g});

    // path found flag
    bool found_goal = false;
    // no solution could be found flag
    bool resign = false;

    while (!found_goal && !resign)
    {
        if (array_open_cell_list.size() == 0)
        {
            return PlanningStatus::Failed;
        }
        // get node with lowest cost
        int curr_cell_x = array_open_cell_list.begin()->pixel.x;
        int curr_cell_y = array_open_cell_list.begin()->pixel.y;
        g = array_open_cell_list.begin()->f_cost;
        array_open_cell_list.erase(array_open_cell_list.begin());

        // check, whether the solution is found (we are at the goal)
        // we stop, when get path to any voronoi cell
        if (voronoi_map.at<uchar>(curr_cell_x, curr_cell_y) == VORONOI_VALUE)
        {
            nearest_point_on_voronoi.x = curr_cell_x;
            nearest_point_on_voronoi.y = curr_cell_y;
            return PlanningStatus::Success;
        }
        else if (curr_cell_x == goal.x && curr_cell_y == goal.y)
        {
            found_goal = true;
            continue;
        }
        for (auto delta : deltas)
        {
            // expansion
            int x2 = curr_cell_x + delta.x;
            int y2 = curr_cell_y + delta.y;
            double l1_cost = delta.x + delta.y;
            double l2_cost = sqrt(pow(delta.x, 2) + pow(delta.y, 2));

            // check new node to be in grid bounds
            if (x2 >= 0 && x2 < voronoi_map.rows && y2 >= 0 && y2 < voronoi_map.cols)
            {
                // check new node not to be in obstacle
                if (obstacle_map.at<uchar>(x2, y2) == 255)
                {
                    continue;
                }
                // check new node was not early visited
                if (visited.at<uchar>(x2, y2) == 255)
                {
                    continue;
                }

                float g2 = g + l2_cost; // movement_cost
                array_open_cell_list.insert({ cv::Point2i(x2, y2), g2 });
                visited.at<uchar>(x2, y2) = 255;
            }
        }
    }
    return PlanningStatus::DirectConnectionToGoal;
}


/** @brief calculates heuristic cost between cells with euclidean distance
 *
 */
float calcHCost(cv::Point2i current_cell, cv::Point2i target_cell, bool use_euclidian_dist)
{
    double l2_dist = std::sqrt(std::pow(current_cell.x - target_cell.x, 2) + std::pow(current_cell.y - target_cell.y, 2));
    double l1_dist = abs(current_cell.x - target_cell.x) + abs(current_cell.y - target_cell.y);
    if (use_euclidian_dist)
    {
        return l2_dist;
    }
    return l1_dist;
}

float calcGCost(float current_cell_g_cost, cv::Point2i current_cell, cv::Point2i target_cell)
{
    return current_cell_g_cost + calcHCost(current_cell, target_cell, false);
}

float calcFCost(float current_cell_g_score, cv::Point2i current_cell, cv::Point2i target_cell, cv::Point2i goal_cell, bool use_euclidean = true)
{
    return calcGCost(current_cell_g_score, current_cell, target_cell) + calcHCost(target_cell, goal_cell, use_euclidean);
}

bool findRelaxedAStarPathOnImage(const cv::Mat& voronoi_map, std::vector<cv::Point2i>& out_path, cv::Point2i start,
                          cv::Point2i goal, bool use_eight_neighbors)
{
    ROS_DEBUG("finding A-Star path on voronoi");
    // TODO: make sure start and goal are valid points and free
    bool start_is_voronoi = voronoi_map.at<uchar>(start.x, start.y) == 255;
    if (!start_is_voronoi)
    {
        ROS_ERROR("Start is not a valid voronoi cell!");
        return false;
    }
    bool goal_is_voronoi = voronoi_map.at<uchar>(goal.x, goal.y) == 255;
    if (!goal_is_voronoi)
    {
        ROS_ERROR("Goal is not a valid voronoi cell!");
        return false;
    }
    // create deltas used for neighborhood check
    std::vector<cv::Point2i> deltas = create_deltas(use_eight_neighbors);
    // crete g_score array
    cv::Mat g_score(voronoi_map.size(), CV_32FC1, std::numeric_limits<float>::infinity());
    g_score.at<float>(start.x, start.y) = 0;
    // fill gscore array with AStar
    std::multiset<AStarCell, std::less<AStarCell>> array_open_cell_list;
    array_open_cell_list.insert({ start, calcHCost(start, goal, false) });

    ROS_DEBUG("Creating g score array");
    int loop_counter = 0;

    bool timeout = false;
    double max_runtime = 4.0;
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    while (!array_open_cell_list.empty() && g_score.at<float>(goal.x, goal.y) == std::numeric_limits<float>::infinity()) //  && !timeout
    {
        loop_counter++;

        // Get cell with lowest f_score and remove it so it will not be visited again
        cv::Point2i current_cell = array_open_cell_list.begin()->pixel;
        array_open_cell_list.erase(array_open_cell_list.begin());
        float current_gscore = g_score.at<float>(current_cell.x, current_cell.y);
        // get all free neighbors
        for (auto delta : deltas)
        {
            cv::Point2i neighbor_cell = current_cell + delta;
            bool is_inside_img = neighbor_cell.x >= 0 && neighbor_cell.x < voronoi_map.rows && neighbor_cell.y >= 0 &&
                                 neighbor_cell.y < voronoi_map.cols;
            if (!is_inside_img)
            {
                continue;
            }
            bool is_free = voronoi_map.at<uchar>(neighbor_cell.x, neighbor_cell.y) == 255;
            if (!is_free)
            {
                continue;
            }
            float neighbor_g_score = calcGCost(current_gscore, current_cell, neighbor_cell);
            // if (neighbor_g_score < g_score.at<float>(neighbor_cell.x, neighbor_cell.y))
            if (g_score.at<float>(neighbor_cell.x, neighbor_cell.y) == std::numeric_limits<float>::infinity())
            {
                g_score.at<float>(neighbor_cell.x, neighbor_cell.y) = calcGCost(current_gscore, current_cell, neighbor_cell);
                array_open_cell_list.insert(
                    { neighbor_cell, calcFCost(current_gscore, current_cell, neighbor_cell, goal) });
            }
        }
        std::chrono::steady_clock::time_point end_loop_time = std::chrono::steady_clock::now();
        double total_time_in_planning = (std::chrono::duration_cast<std::chrono::microseconds>(end_loop_time - start_time).count()) / 1000000.0;
        if (total_time_in_planning > max_runtime)
        {
            timeout = true;
        }
    }
    if (timeout)
    {
        ROS_ERROR("AStar timed out, possibly no path possible");
        return false;
    }
    if (g_score.at<float>(goal.x, goal.y) == std::numeric_limits<float>::infinity())
    {
        ROS_ERROR("No connection on voronoi found");
        return false;
    }
    ROS_DEBUG_STREAM("Loops needed for gscores: " << loop_counter);
    // Construct path backwards from goal to start to get best path
    ROS_DEBUG("Trace back path");
    out_path.clear();
    out_path.push_back(goal);
    cv::Point2i current_cell = goal;
    // bool is_start_cell = (current_cell.x == start.x) && (current_cell.y == start.y);
    // bool is_not_start_cell = current_cell != start;
    while (!((current_cell.x == start.x) && (current_cell.y == start.y)))
    {
        cv::Point2i min_g_score_cell = current_cell;
        ROS_DEBUG_STREAM("Current cell: " << current_cell);
        for (auto delta : deltas)
        {
            cv::Point2i neighbor_cell = current_cell + delta;
            ROS_DEBUG_STREAM("Neighbor cell: " << neighbor_cell);
            bool is_inside_img = neighbor_cell.x >= 0 && neighbor_cell.x < voronoi_map.rows && neighbor_cell.y >= 0 &&
                                 neighbor_cell.y < voronoi_map.cols;
            if (!is_inside_img)
            {
                ROS_DEBUG("Skipped cos not within map");
                continue;
            }
            bool is_free = voronoi_map.at<uchar>(neighbor_cell.x, neighbor_cell.y) == 255;
            if (!is_free)
            {
                ROS_DEBUG("Skipped cos not on voronoi");
                continue;
            }
            if (g_score.at<float>(min_g_score_cell.x, min_g_score_cell.y) > g_score.at<float>(neighbor_cell.x, neighbor_cell.y))
            {
                min_g_score_cell = neighbor_cell;
            }
        }
        out_path.push_back(min_g_score_cell);
        current_cell = min_g_score_cell;
    }
    ROS_DEBUG("reversing path");
    std::reverse(out_path.begin(), out_path.end());
    ROS_DEBUG("Done");
    return true;
}


bool findAStarPathOnImage(const cv::Mat& voronoi_map, std::vector<cv::Point2i>& out_path, cv::Point2i start,
                          cv::Point2i goal, bool use_eight_neighbors)
{
    bool start_is_voronoi = voronoi_map.at<uchar>(start.x, start.y) == 255;
    if (!start_is_voronoi)
    {
        ROS_ERROR("Start is not a valid voronoi cell!");
        return false;
    }
    bool goal_is_voronoi = voronoi_map.at<uchar>(goal.x, goal.y) == 255;
    if (!goal_is_voronoi)
    {
        ROS_ERROR("Goal is not a valid voronoi cell!");
        return false;
    }

    std::vector<cv::Point2i> deltas = create_deltas(use_eight_neighbors);
    std::vector<cv::Point2i> closed_list;
    cv::Mat g_score(voronoi_map.size(), CV_32FC1, std::numeric_limits<float>::infinity());
    g_score.at<float>(start.x, start.y) = 0;
    cv::Mat reached_with_delta_index(voronoi_map.size(), CV_8UC1, cv::Scalar(255));
    cv::Mat visited(voronoi_map.size(), CV_8UC1, cv::Scalar(0));
    visited.at<uchar>(start.x, start.y) = 255;
    // vector of open cells (for possible expansion) nodes
    std::multiset<AStarCell, std::less<AStarCell>> array_open_cell_list;
    array_open_cell_list.insert({ start, calcHCost(start, goal, true) });

    // path found flag
    bool found_goal = false;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    while (!found_goal)
    {
        if (array_open_cell_list.size() == 0)
        {
            ROS_ERROR("No connection on voronoi found");
            return false;
        }
        // get node with lowest cost
        cv::Point2i current_cell = array_open_cell_list.begin()->pixel;
        array_open_cell_list.erase(array_open_cell_list.begin());
        float current_gscore = g_score.at<float>(current_cell.x, current_cell.y);
        // ROS_INFO_STREAM("len of open cells: " << array_open_cell_list.size());
        closed_list.push_back(current_cell);
        visited.at<uchar>(current_cell.x, current_cell.y) = 255;

        if (current_cell.x == goal.x && current_cell.y == goal.y)
        {
            ROS_INFO("Found goal with A*");
            found_goal = true;
            break;
        }

        for (int i = 0; i < deltas.size(); i++)
        {
            cv::Point2i neighbor_cell = current_cell + deltas[i];
            /*
            if (std::find(closed_list.begin(), closed_list.end(), neighbor_cell) != closed_list.end())
            {
                continue;
            }
            */
            bool is_inside_img = neighbor_cell.x >= 0 && neighbor_cell.x < voronoi_map.rows && neighbor_cell.y >= 0 &&
                                 neighbor_cell.y < voronoi_map.cols;
            if (!is_inside_img)
            {
                continue;
            }
            bool is_free = voronoi_map.at<uchar>(neighbor_cell.x, neighbor_cell.y) == 255;
            if (!is_free)
            {
                continue;
            }
            float neighbor_g_score = calcGCost(current_gscore, current_cell, neighbor_cell);
            float neighbor_f_score = calcFCost(current_gscore, current_cell, neighbor_cell, goal, true);
            // std::tuple<float, int, int> neighbor_tuple = {neighbor_f_score, neighbor_cell.x, neighbor_cell.y};
            if (g_score.at<float>(neighbor_cell.x, neighbor_cell.y) == std::numeric_limits<float>::infinity() || neighbor_g_score < g_score.at<float>(neighbor_cell.x, neighbor_cell.y))
            {
                g_score.at<float>(neighbor_cell.x, neighbor_cell.y) = neighbor_g_score;
                array_open_cell_list.insert({neighbor_cell, neighbor_f_score});
                reached_with_delta_index.at<uchar>(neighbor_cell.x, neighbor_cell.y) = i;
            }
        }
    }
    ROS_INFO("Filling gscore done");
    std::chrono::steady_clock::time_point gscore_done = std::chrono::steady_clock::now();
    double gscore_duration = (std::chrono::duration_cast<std::chrono::microseconds>(gscore_done - begin).count()) / 1000000.0;
    ROS_INFO_STREAM("Iteration took (s): " << gscore_duration);
    ROS_INFO("Trace back path");
    out_path.clear();
    out_path.push_back(goal);
    cv::Point2i current_cell = goal;
    while (!((current_cell.x == start.x) && (current_cell.y == start.y)))
    {
        cv::Point2i min_g_score_cell = current_cell;
        ROS_DEBUG_STREAM("Current cell: " << current_cell);
        for (auto delta : deltas)
        {
            cv::Point2i neighbor_cell = current_cell + delta;
            ROS_DEBUG_STREAM("Neighbor cell: " << neighbor_cell);
            bool is_inside_img = neighbor_cell.x >= 0 && neighbor_cell.x < voronoi_map.rows && neighbor_cell.y >= 0 &&
                                 neighbor_cell.y < voronoi_map.cols;
            if (!is_inside_img)
            {
                ROS_DEBUG("Skipped cos not within map");
                continue;
            }
            bool is_free = voronoi_map.at<uchar>(neighbor_cell.x, neighbor_cell.y) == 255;
            if (!is_free)
            {
                ROS_DEBUG("Skipped cos not on voronoi");
                continue;
            }
            if (g_score.at<float>(min_g_score_cell.x, min_g_score_cell.y) > g_score.at<float>(neighbor_cell.x, neighbor_cell.y))
            {
                min_g_score_cell = neighbor_cell;
            }
        }
        out_path.push_back(min_g_score_cell);
        current_cell = min_g_score_cell;
    }
    return true;
}


bool findCompletePath(const cv::Mat& obstacle_img, const cv::Mat& voronoi_img, std::vector<cv::Point2i>& path, cv::Point2i start, cv::Point2i goal)
{
    int connect_goal_success = PlanningStatus::Failed;
    int connect_start_success = PlanningStatus::Failed;

    // if goal is not on voronoi graph, compute shortest path to voronoi graph
    bool start_is_voronoi = voronoi_img.at<uchar>(start.x, start.y) == 255;
    bool goal_is_voronoi = voronoi_img.at<uchar>(goal.x, goal.y) == 255;
    std::chrono::steady_clock::time_point path_start_time = std::chrono::steady_clock::now();

    cv::Point2i start_on_voronoi;
    cv::Point2i goal_on_voronoi;
    if (!goal_is_voronoi)
    {
        connect_goal_success = findDijkstraPathToVoronoi(
            obstacle_img, voronoi_img, goal_on_voronoi, goal, start,
            false);
    }
    else
    {
        goal_on_voronoi = goal;
        connect_goal_success = PlanningStatus::Success;
    }
    std::chrono::steady_clock::time_point path_goal_connection_time = std::chrono::steady_clock::now();
    ROS_INFO_STREAM(
        "PathFinding: Time taken for goal connection (s): "
        << (std::chrono::duration_cast<std::chrono::milliseconds>(path_goal_connection_time - path_start_time)
                .count()) /
               1000.0);

    // if start is not on voronoi graph, compute shortest path to voronoi graph
    if (!start_is_voronoi)
    {
        connect_start_success = findDijkstraPathToVoronoi(
            obstacle_img, voronoi_img, start_on_voronoi, start, goal,
            false);
    }
    else
    {
        start_on_voronoi = start;
        connect_start_success = PlanningStatus::Success;
    }

    if (connect_start_success == PlanningStatus::DirectConnectionToGoal || connect_goal_success == PlanningStatus::DirectConnectionToGoal)
    {
        // skip planning and only use start and goal
        path.push_back(start);
        path.push_back(goal);
    }

    if (!(connect_start_success == PlanningStatus::Success && connect_goal_success == PlanningStatus::Success))
    {
        ROS_ERROR("Failed to connect start or goal to voronoi");
        return false;
    }

    std::chrono::steady_clock::time_point path_start_connection_time = std::chrono::steady_clock::now();
    ROS_INFO_STREAM("PathFinding: Time taken for start connection (s): "
                    << (std::chrono::duration_cast<std::chrono::microseconds>(path_start_connection_time -
                                                                              path_goal_connection_time)
                            .count()) /
                           1000000.0);
    // find path on voronoi
    bool on_voronoi_success = findRelaxedAStarPathOnImage(voronoi_img, path, start_on_voronoi, goal_on_voronoi, true);

    std::chrono::steady_clock::time_point path_complete_time = std::chrono::steady_clock::now();
    ROS_INFO_STREAM(
        "PathFinding: Time taken for on voronoi connection (s): "
        << (std::chrono::duration_cast<std::chrono::milliseconds>(path_complete_time - path_start_connection_time)
                .count()) /
               1000.0);

    if (!on_voronoi_success)
    {
        ROS_ERROR("Failed to find path on voronoi");
        return false;
    }
    // path.emplace(path.begin(), start);
    // path.push_back(goal);
    return true;
}


bool sparsify_path(const std::vector<cv::Point2i>& path_in, std::vector<cv::Point2i>& path_out, float angle_threshold,
                   float min_distance_control_points_px)
{
    if (path_in.size() < 3)
    {
        path_out = path_in;
        ROS_WARN("path was too short for sparsify");
        return false;
    }
    // create plan from path
    cv::Point2i last_added_point;
    double last_yaw = 0.0;
    for (int path_idx = 0; path_idx < path_in.size(); path_idx++)
    {
        cv::Point2i this_point = path_in.at(path_idx);
        // Calculate orientation for each point of the plan with the current position and the last one
        if (path_idx == path_in.size() - 1)
        {
            // Last point in path will always be added
        }
        else if (path_idx == 0)
        {
            // First point will always be added
            // calculate start yaw angle
            // No previous point so orientation will be calculated to next point
            cv::Point2i next_point = path_in.at(path_idx + 1);
            cv::Point2i delta = next_point - this_point;
            double yaw_angle = std::atan2(delta.y, delta.x);
            last_yaw = yaw_angle;
        }
        else
        {
            // Some other points are before and behind, so orientation can be calculated
            float distance_to_last_added =
                sqrt(pow(this_point.x - last_added_point.x, 2) + pow(this_point.y - last_added_point.y, 2));
            cv::Point2i goal_point = path_in.back();
            float distance_to_goal = sqrt(pow(goal_point.x - this_point.x, 2) + pow(goal_point.y - this_point.y, 2));
            cv::Point2i next_point = path_in.at(path_idx + 1);
            cv::Point2i delta = next_point - this_point;
            double yaw_angle = std::atan2(delta.y, delta.x);
            // double angle_diff = M_PI - fabs(fmod(fabs(yaw_angle - last_yaw), 2 * M_PI) - M_PI);

            int num_samples = std::min(3, (int) (path_in.size() - path_idx - 1));
            double sum_sins = 0.0;
            double sum_coss = 0.0;
            for (int forward_idx = path_idx + 1; forward_idx < path_idx + 1 + num_samples; forward_idx++)
            {
                next_point = path_in.at(forward_idx);
                delta = next_point - this_point;
                yaw_angle = std::atan2(delta.y, delta.x);
                sum_sins += std::sin(yaw_angle);
                sum_coss += std::cos(yaw_angle);
            }
            double avg_angle = std::atan2(sum_sins / (double)num_samples, sum_coss / (double)num_samples);
            double angle_diff = std::min(2 * M_PI - abs(avg_angle - last_yaw), abs(avg_angle - last_yaw));
            if (angle_diff < angle_threshold || distance_to_last_added < min_distance_control_points_px ||
                distance_to_goal < min_distance_control_points_px)
            {
                // Dont include points with small angular change
                // ROS_INFO("Dont include this point due to small angular change");
                continue;
            }
            last_yaw = avg_angle;
        }
        last_added_point = this_point;
        path_out.push_back(this_point);
    }
    ROS_DEBUG_STREAM("Reduced amount of points from " << path_in.size() << " to " << path_out.size());
    return true;
}



void mapToWorld(const cv::Point2i& point_map, cv::Point2d& point_world, cv::Point2d map_origin, double resolution)
{
    double half_a_cell = 0.5;
    point_world.x = map_origin.x + (point_map.x + half_a_cell) * resolution;
    point_world.y = map_origin.y + (point_map.y + half_a_cell) * resolution;
}

bool worldToMap(const cv::Point2d& point_world, cv::Point2i& point_map, cv::Point2d map_origin, int map_size_x,
                int map_size_y, double resolution)
{
    if (point_world.x < map_origin.x || point_world.y < map_origin.y)
        return false;

    point_map.x = (int)((point_world.x - map_origin.x) / resolution);
    point_map.y = (int)((point_world.y - map_origin.y) / resolution);

    if (point_map.x < map_size_x && point_map.y < map_size_y)
        return true;

    return false;
}

bool isPlanFree(std::shared_ptr<costmap_2d::Costmap2D> costmap, int free_cell_threshold,
                const std::vector<geometry_msgs::PoseStamped>& plan)
{
    cv::Point2d map_origin(costmap->getOriginX(), costmap->getOriginY());
    int map_size_x = costmap->getSizeInCellsX();
    int map_size_y = costmap->getSizeInCellsY();
    double map_resolution = costmap->getResolution();
    for (auto pose : plan)
    {
        cv::Point2i point_map;
        worldToMap(cv::Point2d(pose.pose.position.x, pose.pose.position.y), point_map, map_origin, map_size_x,
                   map_size_y, map_resolution);

        double cell_cost = costmap->getCost(point_map.x, point_map.y);
        if (cell_cost > free_cell_threshold && cell_cost < 255)
        {
            ROS_WARN_STREAM("Plan is not free at " << point_map.x << ", " << point_map.y);
            return false;
        }
    }
    // ROS_WARN("Plan is free!");
    return true;
}


void createPlanFromPath(const std::vector<cv::Point2i>& path_in, std::vector<geometry_msgs::PoseStamped>& plan_out,
                        std::shared_ptr<costmap_2d::Costmap2D> costmap, std::string global_frame_id)
{
    double half_a_cell = 0.5;
    cv::Point2d map_origin(costmap->getOriginX(), costmap->getOriginY());
    int map_size_x = costmap->getSizeInCellsX();
    int map_size_y = costmap->getSizeInCellsY();
    double map_resolution = costmap->getResolution();

    std::vector<cv::Point2d> path_world;

    for (auto point_map : path_in)
    {
        cv::Point2d point_world;
        mapToWorld(point_map, point_world, map_origin, map_resolution);
        path_world.push_back(point_world);
    }
    createPlanFromPath(path_world, plan_out, global_frame_id);
}

void createPlanFromPath(const std::vector<cv::Point2d>& path_in, std::vector<geometry_msgs::PoseStamped>& plan_out, std::string global_frame_id)
{
    std::vector<cv::Point3d> poses_world;
    for (int path_idx = 0; path_idx < path_in.size(); path_idx++)
    {
        cv::Point2d this_point = path_in.at(path_idx);

        cv::Point3d this_pose;
        this_pose.x = this_point.x;
        this_pose.y = this_point.y;

        if (path_idx == path_in.size() - 1)
        {
            if (path_in.size() == 1)
            {
                // cant compute orientation from one point, assume 0 rotation
                this_pose.z = 0.0;
            }
            else
            {
                // Last point in path so orientation of last pose will be taken
                this_pose.z = poses_world.back().z;
            }
        }
        else
        {
            // Some other points are behind, so orientation can be calculated
            cv::Point2d next_point = path_in.at(path_idx + 1);
            cv::Point2d delta = next_point - this_point;
            double yaw_angle = std::atan2(delta.y, delta.x);
            this_pose.z = yaw_angle;
        }
        poses_world.push_back(this_pose);
    }
    createPlanFromPath(poses_world, plan_out, global_frame_id);
}

void createPlanFromPath(const std::vector<cv::Point3d>& path_in, std::vector<geometry_msgs::PoseStamped>& plan_out, std::string global_frame_id)
{
    for (auto pose: path_in)
    {
        geometry_msgs::PoseStamped curr_pose;
        curr_pose.header.frame_id = global_frame_id;
        curr_pose.header.stamp = ros::Time::now();
        curr_pose.pose.position.x = pose.x;
        curr_pose.pose.position.y = pose.y;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, pose.z);
        curr_pose.pose.orientation = tf2::toMsg(quat);
        plan_out.push_back(curr_pose);
    }
}


void pathMapToWorld(const std::vector<cv::Point2i>& path_in, std::vector<cv::Point2d>& path_world_out,
                    std::shared_ptr<costmap_2d::Costmap2D> costmap)
{
    cv::Point2d map_origin(costmap->getOriginX(), costmap->getOriginY());
    int map_size_x = costmap->getSizeInCellsX();
    int map_size_y = costmap->getSizeInCellsY();
    double map_resolution = costmap->getResolution();

    for (auto point_map : path_in)
    {
        cv::Point2d point_world;
        mapToWorld(point_map, point_world, map_origin, map_resolution);
        path_world_out.push_back(point_world);
    }
}

} // path_planning

#pragma once

#include <iostream>
#include <chrono>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace PlanningStatus
{
enum PlanningStatus
{
    Success = 0,
    Failed = -1,
    DirectConnectionToGoal = 1
};
}

namespace path_planning
{


/**
 * @brief Create all deltas to get neighbors of a pixel with spefified neighborhood type
 *
 * @param use_eight_neighbors true for using 8-neighbors, false for 4-neighbors
 * @return vector of delta points
 */
std::vector<cv::Point2i> create_deltas(bool use_eight_neighbors);

static int OBSTACLE_VALUE = 0;
static int FREE_SPACE_THRESH = 127;
static int VORONOI_VALUE = 255;

/**
 * @brief finds dijkstra path from start to voronoi map.
 *
 * @param obstacle_map input obstacle map with 255 as occupied cells and 0 as free cells
 * @param voronoi_map input voronoi map with 255 as voronoi cells, 127 as free cells and 0 as occupied cells
 * @param nearest_point_on_voronoi output point on voronoi which is closest to start
 * @param start start point
 * @param goal goal point
 * @param use_eight_neighbors if 8 connecting neighbors are used for exploration; 4 otherwise
 * @return int status code of path planning
 */
int findDijkstraPathToVoronoi(const cv::Mat& obstacle_map, const cv::Mat& voronoi_map,
                               cv::Point2i& nearest_point_on_voronoi, cv::Point2i start, cv::Point2i goal,
                               bool use_eight_neighbors);


struct AStarCell
{
    cv::Point2i pixel;
    float f_cost;

    bool operator<(const AStarCell& rhs) const
    {
        return this->f_cost < rhs.f_cost;
    }

    bool operator>(const AStarCell& rhs) const
    {
        return this->f_cost > rhs.f_cost;
    }
};

/**
 * @brief finds relaxed A* path from start to goal on voronoi map.
 *
 * @param voronoi_map map with 255 as voronoi cells, 127 as free cells and 0 as occupied cells
 * @param out_path output path on voronoi map
 * @param start start point
 * @param goal goal point
 * @param use_eight_neighbors if 8 connecting neighbors are used for exploration; 4 otherwise
 * @return true if path was found;
 * @return false otherwise
 */
bool findRelaxedAStarPathOnImage(const cv::Mat& voronoi_map, std::vector<cv::Point2i>& out_path, cv::Point2i start,
                          cv::Point2i goal, bool use_eight_neighbors);


/**
 * @brief finds complete path from start to goal
 *
 * three components: start to voronoi; on voronoi; voronoi to goal
 *
 * @param obstacle_img map with 255 as free cells and 0 as occupied cells
 * @param voronoi_img map with 255 as voronoi cells, 127 as free cells and 0 as occupied cells
 * @param path output path
 * @param start start point
 * @param goal goal point
 * @return true if path was found;
 * @return false otherwise
 */
bool findCompletePath(const cv::Mat& obstacle_img, const cv::Mat& voronoi_img, std::vector<cv::Point2i>& path, cv::Point2i start, cv::Point2i goal);

/**
 * @brief creates sparse path from full path by selecting only relevant points where orientation changes
 *
 * @param path_in input dense path
 * @param path_out output sparse path
 * @param angle_threshold angular change for adding points
 * @param min_distance_control_points_px minimum distance between two control points in sparse path
 * @return true if sparsify successful;
 * @return false when input path is too short for sparsification
 */
bool sparsify_path(const std::vector<cv::Point2i>& path_in, std::vector<cv::Point2i>& path_out, float angle_threshold,
                   float min_distance_control_points_px);

/**
 * @brief transforms point from map coordinates in world coordinates
 *
 * @param point_map input point in map frame (positive integers)
 * @param point_world output point in world frame (double)
 * @param map_origin origin of map, see costmap->getOriginX() and costmap->getOriginY()
 * @param resolution resolution of map (m/px), see costmap->getResolution()
 */
void mapToWorld(const cv::Point2i& point_map, cv::Point2d& point_world, cv::Point2d map_origin, double resolution);

/**
 * @brief transforms point from world coordinates in map coordinates
 *
 * @param point_world input point in world frame (double)
 * @param point_map output point in map frame (positive integers)
 * @param map_origin origin of map, see costmap->getOriginX() and costmap->getOriginY()
 * @param map_size_x map size x in pixel, see costmap->getSizeInCellsX()
 * @param map_size_y map size y in pixel, see costmap->getSizeInCellsY()
 * @param resolution resolution of map (m/px), see costmap->getResolution()
 * @return true if transformed point is within bounds of map (positive)
 * @return false otherwise
 */
bool worldToMap(const cv::Point2d& point_world, cv::Point2i& point_map, cv::Point2d map_origin, int map_size_x,
                int map_size_y, double resolution);


/**
 * @brief Convertes Path to ROS Message Type for Plan; fills in orientation for each position
 *
 * @param path_in input path as vector of points in map coordinates
 * @param plan_out output plan as vector of PoseStamped
 * @param costmap map for coordinate transformation
 * @param global_frame_id map frame id
 */
void createPlanFromPath(const std::vector<cv::Point2i>& path_in, std::vector<geometry_msgs::PoseStamped>& plan_out,
                        std::shared_ptr<costmap_2d::Costmap2D> costmap, std::string global_frame_id);

/**
 * @brief Convertes Path to ROS Message Type for Plan; fills in orientation for each position
 *
 * @param path_in input path as vector of points in world coordinates
 * @param plan_out output plan as vector of PoseStamped
 * @param global_frame_id map frame id
 */
void createPlanFromPath(const std::vector<cv::Point2d>& path_in, std::vector<geometry_msgs::PoseStamped>& plan_out, std::string global_frame_id);

/**
 * @brief Convertes Path to ROS Message Type for Plan
 *
 * @param path_in input path as vector of points in world coordinates with orientation
 * @param plan_out output plan as vector of PoseStamped
 * @param global_frame_id map frame id
 */
void createPlanFromPath(const std::vector<cv::Point3d>& path_in, std::vector<geometry_msgs::PoseStamped>& plan_out, std::string global_frame_id);

/**
 * @brief converts all points in path to world frame and calculates angles.
 *
 * @param path_in input path in map space (pixels)
 * @param path_world_out output path in world space (meters)
 * @param costmap costmap for conversion properties
 */
void pathMapToWorld(const std::vector<cv::Point2i>& path_in, std::vector<cv::Point2d>& path_world_out,
                    std::shared_ptr<costmap_2d::Costmap2D> costmap);


/**
 * @brief Checks if costmap is free at every pose in plan.
 *
 * @param costmap obstacle map
 * @param free_cell_threshold threshold for free cells
 * @param plan plan which will be checked
 * @return true if plan is free;
 * @return false otherwise
 */
bool isPlanFree(std::shared_ptr<costmap_2d::Costmap2D> costmap, int free_cell_threshold,
                const std::vector<geometry_msgs::PoseStamped>& plan);


} // path_planning

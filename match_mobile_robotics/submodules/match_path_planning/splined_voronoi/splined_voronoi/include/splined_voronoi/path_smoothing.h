#pragma once

#include <iostream>
#include <numeric>
#include <chrono>
// #include <tinysplinecxx.h>
#include <nlopt.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <costmap_2d/costmap_2d.h>

namespace path_smoothing
{

namespace OptimizationStatus
{
enum OptimizationStatus
{
    Success = 0,
    Failure = -1,
    TooFewPoints = 1,
    NoOptimizationNeeded = 2
};
}

/**
 * @brief container for data which is needed for optimization
 *
 */
typedef struct {
    const std::vector<cv::Point2d>& points_orig;
    const std::vector<int>& optimize_indices;
    const costmap_2d::Costmap2D& costmap;
    const cv::Mat& costmap_img;
    double curvature_limit;
    double time_limit;
    std::chrono::steady_clock::time_point start_time;
    bool optimize_lengths;
    double default_length;
    bool was_terminated;
    std::vector<double>& best_result;
    const std::vector<int>& optimize_lengths_indices;
} specific_points_optim_data;

/**
 * @brief interpolates sparse path with splines from tinyspline. Performs Optimization so that curvature is limited and no obstacles are hit.
 *
 * @param sparse_path input sparse path which contains corner points for splines
 * @param path_out output path which should be continuous if input was correct
 * @param optimized_sparse_path output sparse path which contains corner points for splines
 * @param optimized_lengths output tangent lengths of spline for each connection point of spline
 * @param costmap costmap for transforming coordinates
 * @param obstacle_img binary image of obstacles in map
 * @param curve_max boulding value for curvature (1 / desired min radius)
 * @param path_resolution wanted resolution of output path in points per meter
 * @param optimize_lengths if tangent lengths should be optimized; recommend true
 * @param max_optimization_time maximum time for optimization in seconds; recommend 4.0
 * @return true if optimized continuous could be calculated;
 * @return false otherwise
 */
bool buildOptimizedContinuousPath(const std::vector<cv::Point2d>& sparse_path, std::vector<cv::Point2d>& path_out, std::vector<cv::Point2d>& optimized_sparse_path, std::vector<double>& optimized_lengths,
                                  const costmap_2d::Costmap2D& costmap, const cv::Mat& obstacle_img, double curve_max, double path_resolution, bool optimize_lengths, double max_optimization_time);


/**
 * @brief samples spline built from waypoints and tangent lengths.
 *
 * @param points input waypoints which contain corner points for spline
 * @param lengths input tangent lengths at connection points of spline
 * @param points_out output samples of spline
 * @param sample_distance approximate distance between two consecutive samples
 * @param default_length default tangent length if tangent length vector is empty
*/
void getSplinePathSamples(const std::vector<cv::Point2d>& points, std::vector<double>& lengths, std::vector<cv::Point2d>& points_out, double sample_distance, double default_length);


}  // namespace path_smoothing

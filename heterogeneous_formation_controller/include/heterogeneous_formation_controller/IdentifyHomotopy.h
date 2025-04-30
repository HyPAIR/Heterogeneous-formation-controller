/**
 * file topo_prm.h
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief topological PRM
 * data 2024-4-2
 * 
 * @copyright Copyroght(c) 2024
*/

#ifndef _IDENTIFYHOMOTOPY_H
#define _IDENTIFYHOMOTOPY_H

#include <random>
#include <iostream>
#include <Eigen/Eigen>
#include <list>
#include <vector>
// #include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <utility>
#include <memory>
#include "heterogeneous_formation_controller/environment.h"
#include "heterogeneous_formation_controller/math/pose.h"
#include "heterogeneous_formation_controller/planner_config.h"

using std::vector;
using std::list;
using std::max;
using std::shared_ptr;
using std::random_device;
using std::default_random_engine;
using std::uniform_real_distribution;

namespace heterogeneous_formation_controller {
struct Point {
    double x, y;
    Point(double x_val, double y_val) : x(x_val), y(y_val) {}
};
struct Pathset {
    double path_length;
    vector<Eigen::Vector2d> path;
};
std::vector<std::vector<std::vector<std::vector<double>>>> CalCorridors(const vector<vector<vector<Eigen::Vector2d>>>& raw_paths_set, 
    std::shared_ptr<heterogeneous_formation_controller::Environment>& env);
std::vector<int> findTwoSmallest(const std::vector<double>& vec);
std::vector<int> evalPathLength(const vector<vector<Eigen::Vector2d>>& raw_paths);
double cross(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);
bool onSegment(const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d &r);
bool segmentsIntersect(const Eigen::Vector2d &p1, const Eigen::Vector2d &q1, const Eigen::Vector2d &p2, const Eigen::Vector2d &q2);
bool lineIntersectsPolygon(const Eigen::Vector2d &a, const Eigen::Vector2d &b, const math::Polygon2d &polygon, Eigen::Vector2d &p1, Eigen::Vector2d &p2);
bool lineVisib(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, std::shared_ptr<heterogeneous_formation_controller::Environment>& env);
}  // namespace fast_planner

#endif
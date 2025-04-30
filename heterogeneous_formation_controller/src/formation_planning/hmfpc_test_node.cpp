/**
* file animation.cpp
* author Weijian Zhang (wxz163@student.bham.ac.uk)
* brief formation planning algorithm test
* data 2023-11-22
* 
* @copyright Copyroght(c) 2023
*/
#include <ros/ros.h>
#include <memory>
#include "heterogeneous_formation_controller/heterogeneous_formation_controller.h"
#include "heterogeneous_formation_controller/visualization/plot.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>

#include "iris/iris.h"
#include "heterogeneous_formation_controller/yaml_all.h"
#include "heterogeneous_formation_controller/math/generate_obs.h"
#include <Eigen/Core>
#include <math.h>
#include <random>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace heterogeneous_formation_controller;

// 遍历文件中的每个数据条目
struct Data {
double x;
double y;
double theta;
};
visualization_msgs::InteractiveMarker CreateMarker(int i, const math::Polygon2d &polygon, double width, visualization::Color c) {
visualization_msgs::InteractiveMarker marker;
marker.header.frame_id = "odom";
marker.header.stamp = ros::Time::now();
marker.name = "Obstacle " + std::to_string(i);
// marker.pose.position.x = polygon.center().x();
// marker.pose.position.y = polygon.center().y();
marker.pose.orientation.w = 1.0;

visualization_msgs::Marker polygon_marker;
polygon_marker.header.frame_id = marker.header.frame_id;
polygon_marker.header.stamp = ros::Time();
polygon_marker.ns = "Obstacles";
polygon_marker.id = i;

polygon_marker.action = visualization_msgs::Marker::ADD;
polygon_marker.type = visualization_msgs::Marker::LINE_STRIP;
polygon_marker.pose.orientation.w = 1.0;
polygon_marker.scale.x = width;
polygon_marker.color = c.toColorRGBA();

for (size_t i = 0; i < polygon.num_points(); i++) {
geometry_msgs::Point pt;
pt.x = polygon.points().at(i).x();
pt.y = polygon.points().at(i).y();
polygon_marker.points.push_back(pt);
}
polygon_marker.points.push_back(polygon_marker.points.front());

visualization_msgs::InteractiveMarkerControl box_control;
box_control.always_visible = true;
box_control.markers.push_back(polygon_marker);

marker.controls.push_back(box_control);

visualization_msgs::InteractiveMarkerControl move_control;
move_control.name = "move_x";
move_control.orientation.w = 0.707107f;
move_control.orientation.x = 0;
move_control.orientation.y = 0.707107f;
move_control.orientation.z = 0;
move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

marker.controls.push_back(move_control);
return marker;
}

void DrawTrajectoryRviz(const FullStates sol_traj,std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config,
int robot_index, ros::Publisher path_pub) {
const std::string robot_name = "Footprint_" + std::to_string(robot_index);
for(int i = 0; i < 1e3; i++) {
visualization::Delete(i, robot_name);
}
visualization::Trigger();

nav_msgs::Path msg;
msg.header.frame_id = "odom";
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
visualization::PlotPolygon(math::Polygon2d(box), 0.1, color, i, robot_name);
}
path_pub.publish(msg);
visualization::Trigger();
}

double getRandomDouble(double min, double max) {
static std::random_device rd;
static std::mt19937 gen(rd());
std::uniform_real_distribution<double> dis(min, max);
return dis(gen);
}

// 生成在[-pi, pi]范围内的随机角度
double getRandomAngle() {
return getRandomDouble(0, M_PI / 2);
}

// 生成随机三维点
TrajectoryPoint generateRandomStartPoint() {
TrajectoryPoint point;
// point.x = getRandomDouble(-20.0, -8.0);
// point.y = getRandomDouble(-20.0, -5.0);
point.x = -30.0;
point.y = getRandomDouble(-30.0, 30.0);
point.theta = getRandomAngle();
// point.theta = 0.0;
return point;
}

TrajectoryPoint generateRandomGoalPoint() {
TrajectoryPoint point;
// point.x = getRandomDouble(8.0, 20.0);
// point.y = getRandomDouble(0.0, 20.0);
point.x = getRandomDouble(-30.0, 30.0);
point.y = 30.0;
// point.theta = M_PI / 2;
point.theta = getRandomAngle();
return point;
}

FullStates traj2fullstates(Trajectory_temp traj) {
FullStates solution;
solution.states.resize(traj.size());
solution.tf = traj[traj.size() - 1].t;
for (int i = 0; i < traj.size(); i++) {
solution.states[i].x = traj[i].x;
solution.states[i].y = traj[i].y;
solution.states[i].theta = traj[i].theta;
solution.states[i].v = traj[i].v;
solution.states[i].phi = traj[i].phi;
solution.states[i].a = traj[i].a;
solution.states[i].omega = traj[i].omega;
}
return solution;
}

void operator>>(const YAML::Node& node, Data& data) {
if (node["x"] && node["y"] && node["theta"]) {
data.x = node["x"].as<double>();
data.y = node["y"].as<double>();
data.theta = node["theta"].as<double>();
} else {
throw std::runtime_error("Invalid node; missing one of 'x', 'y', or 'theta'");
}
}

void DrawTrajectoryRvizSE(const FullStates sol_traj, std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config,
int robot_index, ros::Publisher path_pub) {
for (int i = 0; i < sol_traj.states.size(); i++) {
auto x0_disc = config->vehicle.GetVertexPositions(sol_traj.states[i].x, sol_traj.states[i].y, sol_traj.states[i].theta, 2.0);
std::vector<double> x1, y1;
x1.push_back(x0_disc[0]);x1.push_back(x0_disc[2]);x1.push_back(x0_disc[4]);x1.push_back(x0_disc[6]);x1.push_back(x0_disc[0]);
y1.push_back(x0_disc[1]);y1.push_back(x0_disc[3]);y1.push_back(x0_disc[5]);y1.push_back(x0_disc[7]);y1.push_back(x0_disc[1]);
auto color = visualization::Color::White;
color.set_alpha(0.03);
visualization::Plot(x1, y1, 0.2, color, i, "spatial_envelopes" + std::to_string(robot_index));
}
}

int main(int argc, char **argv) {
ros::init(argc, argv, "hmfpc_test_node");

auto config_ = std::make_shared<PlannerConfig>();
config_->vehicle.InitializeDiscs();

auto env = std::make_shared<Environment>(config_);
auto planner_ = std::make_shared<hmfpcLocalPlanner>(config_, env);

ros::NodeHandle nh;
ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/hmfpc_test_path", 1, false);
ros::Publisher path_pub_diff1 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_diff1", 1, false);
ros::Publisher path_pub_diff2 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_diff2", 1, false);
ros::Publisher path_pub_diff3 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_diff3", 1, false);
ros::Publisher path_pub_diff4 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_diff4", 1, false);
ros::Publisher path_pub_car1 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_car1", 1, false);
ros::Publisher path_pub_car2 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_car2", 1, false);

interactive_markers::InteractiveMarkerServer server_("/hmfpc_obstacle");
math::GenerateObstacle generateobs;
std::vector<math::Pose> obstacle;
std::vector<math::Polygon2d> polys, polys_orig;
std::vector<std::vector<math::Vec2d>> poly_vertices_set;

// double inflated_radius;
// config_->formation.calculateMinCircle(inflated_radius, 0.0, 0.0);
// for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
// polys.push_back(math::Polygon2d(env->InflateObstacle(poly_vertices_set[obs_ind], inflated_radius)));
// }
iris::IRISProblem iris_problem(2);
Eigen::MatrixXd obs = Eigen::MatrixXd::Zero(2, 4);
// polys.push_back(math::Polygon2d({{ -34, 45}, {-35, 33}, {-11, 32}, {-12, 44}})); 
polys.push_back(math::Polygon2d({{-45.379, 19.929}, {-43.762, 7.593}, {-38.704, 6.749}, {-38.380, 21.839}}));
polys.push_back(math::Polygon2d({{26.8, 29.2}, {28.6, 23.4}, {38.6, 23.6}, {38.8, 29}}));
polys.push_back(math::Polygon2d({{-32.599,28.812}, {-32.72,28.232}, {-19.6,28.232}, {-19.823,30.048}}));
polys.push_back(math::Polygon2d({{-19.48,28.538}, {-19.505,20.643}, {-1.394,20.469}, {-0.785,29.291}}));
polys.push_back(math::Polygon2d({{-33.526, 29.015}, {-33.529, 28.050}, {-32.508, 28.033}, {-32.602, 28.999}}));
polys.push_back(math::Polygon2d({{-33.510, 21.000}, {-33.499, 20.034}, {-32.479, 19.990}, {-32.544, 20.997}}));
polys.push_back(math::Polygon2d({{-20.058, 28.539}, {-20.007, 27.595}, {-19.045, 27.574}, {-18.989, 28.483}}));
polys.push_back(math::Polygon2d({{20.011, 20.914}, {-20.014, 20.024}, {-19.016, 20.056}, {-19.031, 21.018}}));
polys.push_back(math::Polygon2d({{ -43.7, 52.9}, {-74.3, 52.7}, {-73.7, 45.2}, {-45.2, 45.5}}));
polys.push_back(math::Polygon2d({{ -74, 52.2}, {-74.3, 10}, {-81.2, 10}, {-81.2, 52.5},})); 
// polys.push_back(math::Polygon2d({{ -74, 10}, {-73.6, 16.1}, {-50, 22.1}, {-50, 10.7}})); 
polys.push_back(math::Polygon2d({{ -49.7, 10}, {-53.1, 10}, {-52.3, 0}, {-50, 0}})); 
polys.push_back(math::Polygon2d({{ -51.883, 21.980}, {-52.203, 11.242}, {-49.861, 11.063}, {-50.055, 22.146}})); 
polys.push_back(math::Polygon2d({{ -49.7, -0.5}, {-49.1, -2}, {6.928, -2}, {6.928, -0.5}})); 
polys.push_back(math::Polygon2d({{ 15, -1}, {15, -2}, {75, -2}, {75, -1}})); 
polys.push_back(math::Polygon2d({{ -42.6, 51.3}, {-1.3, 51.1}, {-1.6, 56}, {-46.1, 56}})); 
polys.push_back(math::Polygon2d({{ -0.65, 52.5}, {-1.3, 8.75}, {24.2, 8}, {23.6, 51.2}})); 
// polys.push_back(math::Polygon2d({{ 39.5 + 1, 20 + 1}, {39.5 - 1, 20 + 1}, {39.5 - 1, 20 - 1}, {39.5 + 1, 20 - 1}})); 
polys.push_back(math::Polygon2d({{ 74, -1.5}, {76.7, -1.5}, {76.7, 50}, {74, 50}})); 
polys.push_back(math::Polygon2d({{ -80.5, 50.1}, {-80.5, -54}, { -82.5, -54}, {-82.5, 50.1}})); 
polys.push_back(math::Polygon2d({{ -80.5, 51}, {75, 51}, {75, 59}, {-80.5, 59}})); 
polys.push_back(math::Polygon2d({{ 75, 50.1}, {75, -54}, {77, -54}, {77, 50.1}})); 
polys.push_back(math::Polygon2d({{ 75, -54}, {75, -56}, {-80.5, -56}, {-80.5, -54}})); 
// polys.push_back(math::Polygon2d({{38.7, 35}, {38.6, 33}, {41, 33}, {42, 34.5}}));
polys.push_back(math::Polygon2d({{56.9, 33.9}, {56.9, 32.48}, {59.4, 32.48}, {59.4, 34}}));
polys.push_back(math::Polygon2d({{39, 21}, {39, 19.7}, {40.8, 19.7}, {41, 21}}));
polys.push_back(math::Polygon2d({{56.9, 21}, {56.9, 19.5}, {59, 19.5}, {59, 21}}));
// polys.push_back(math::Polygon2d({{56.9, 21}, {56.9, 19.8}, {59, 19.7}, {59, 21}}));
polys.push_back(math::Polygon2d({{38.998, 34.505}, {38.972, 32.482}, {41.015, 32.500}, {40.996, 34.498}}));
polys.push_back(math::Polygon2d({{42.109, 50.589}, {42.397, 46.760}, {44.048, 46.604}, {43.911, 50.508}}));
polys.push_back(math::Polygon2d({{51.458, 50.486}, {51.530, 46.680}, {52.974, 46.536}, {52.988, 50.787}}));
polys.push_back(math::Polygon2d({{47.041, 50.584}, {46.956, 50.084}, {47.456, 50.110}, {47.430, 50.638}}));
polys.push_back(math::Polygon2d({{60.946, 49.959}, {60.997, 46.201}, {62.008, 46.198}, {61.926, 50.145}}));
polys.push_back(math::Polygon2d({{52.929, 50.742}, {53.053, 50.078}, {74.870, 49.946}, {74.969, 51.240}}));
polys.push_back(math::Polygon2d({{-69.557, 15.067}, {-69.569, 11.240}, {-68.484, 11.236}, {-68.472, 14.950}}));
polys.push_back(math::Polygon2d({{-73.797, 11.082}, {-73.743, 9.996}, {-50.206, 10.436}, {-50.116, 11.451}}));


// for(int i = 0; i < env_->points().size(); i++) {
// obs << env_->points()[i].x() + 0.25, env_->points()[i].x() - 0.25, env_->points()[i].x() - 0.25, env_->points()[i].x() + 0.25,
// env_->points()[i].y() + 0.25, env_->points()[i].y() + 0.25, env_->points()[i].y() - 0.25, env_->points()[i].y() - 0.25;
// iris_problem.addObstacle(obs);
// }
// add boundary
// obs << -52, -52, -50, -50,
// -50, 50, 50, -50;
// iris_problem.addObstacle(obs);
// obs << -50, -50, 50, 50,
// 50, 52, 52, 50;
// iris_problem.addObstacle(obs);
// obs << 50, 52, 52, 50,
// 50, 50, -50, -10;
// iris_problem.addObstacle(obs);
// obs << -50, -50, 50, 50,
// -50, -52, -52, -50;
// iris_problem.addObstacle(obs);
// std::vector<math::Polygon2d> polys = {
// math::Polygon2d({{-3, -3}, {-3, 3}, {3, 3}, {3, -3}}),
// };
// polys.push_back(math::Polygon2d({{-130.5, -130}, {-130.5, 130}, {-130, 130}, {-130, -130}}));
// polys.push_back(math::Polygon2d({{-130, 130}, {-130, 130.5}, {130, 130.5}, {130, 130}}));
// polys.push_back(math::Polygon2d({{130, 130}, {130.5, 130}, {130.5, -130}, {130, -130}}));
// polys.push_back(math::Polygon2d({{-130, -130}, {-130, -130.5}, {130, -130.5}, {130, -130}}));

// polys.push_back(math::Polygon2d({{-10, -5.5}, {-10, 5.5}, {10, 5.5}, {10, -5.5}}));
// polys.push_back(math::Polygon2d({{-30.5, -30}, {-30.5, 30}, {-30, 30}, {-30, -30}}));
// polys.push_back(math::Polygon2d({{-30, 30}, {-30, 30.5}, {30, 30.5}, {30, 30}}));
// polys.push_back(math::Polygon2d({{30, 30}, {30.5, 30}, {30.5, -30}, {30, -30}}));
// polys.push_back(math::Polygon2d({{-30, -30}, {-30, -30.5}, {30, -30.5}, {30, -30}}));
// polys.push_back(math::Polygon2d({{-10, -0.5}, {-10, 0.5}, {10, 0.5}, {10, -0.5}}));

// obstacle.push_back({-1.5, 1.5, 0});
// polys.push_back(math::Polygon2d({{-70, -2}, {30, -2}, {30, -30}, {-70, -30}}));

// polys.push_back(math::Polygon2d({{-14, 15}, {-12, 15}, {-12, 17}, {-14, 17}}));
// polys.push_back(math::Polygon2d({{-14, 10}, {-12, 10}, {-12, 12}, {-14, 12}}));
// polys.push_back(math::Polygon2d({{-28, 10}, {-26, 10}, {-26, 12}, {-28, 12}}));
// polys.push_back(math::Polygon2d({{-28, 15}, {-26, 15}, {-26, 17}, {-28, 17}}));

// polys.push_back(math::Polygon2d({{15, 15}, {17, 15}, {17, 17}, {15, 17}}));
// polys.push_back(math::Polygon2d({{26, 15}, {28, 15}, {28, 17}, {26, 17}}));
// polys.push_back(math::Polygon2d({{15, 26}, {17, 26}, {17, 28}, {15, 28}}));
// polys.push_back(math::Polygon2d({{26, 26}, {28, 26}, {28, 28}, {26, 28}}));

// obstacle.push_back({0, 11.5, 0});
// obstacle.push_back({0, -11.5, 0});
// polys.push_back(math::Polygon2d({{-30.5, -30}, {-30.5, 30}, {-30, 30}, {-30, -30}}));
// polys.push_back(math::Polygon2d({{-30, 30}, {-30, 30.5}, {30, 30.5}, {30, 30}}));
// polys.push_back(math::Polygon2d({{30, 30}, {30.5, 30}, {30.5, -30}, {30, -30}}));
// polys.push_back(math::Polygon2d({{-30, -30}, {-30, -30.5}, {30, -30.5}, {30, -30}}));
// polys.push_back(math::Polygon2d({{-27, 25}, {-23, 20}, {-17, 21}, {-15, 24}}));
// polys.push_back(math::Polygon2d({{-20, 14.4}, {-16, 9}, {-6, 9.6}, {-14, 16}}));
// polys.push_back(math::Polygon2d({{-17, 0}, {-18, -6}, {-6, -7.6}, {-5.3, -1.9}}));
// polys.push_back(math::Polygon2d({{5, -8.7}, {3.7, -12.6}, {14, -20}, {15, -10}}));
// polys.push_back(math::Polygon2d({{5.6, 13.6}, {3.7, 7.8}, {17, 6.8}, {14.2, 9.6}}));
// polys.push_back(math::Polygon2d({{16.3, 27.8}, {16.5, 25}, {27.3, 25}, {27.3, 27.9}}));
// polys.push_back(math::Polygon2d({{-28.4, 3.9}, {-28.3, -3.8}, {-22.6, -4}, {-20.6, 0.5}}));
// polys.push_back(math::Polygon2d({{-27.8, -23}, {-26.4, -28.5}, {-16.2, -28.3}, {-17.5, -24.7}}));
// polys.push_back(math::Polygon2d({{23.2, -16.4}, {15.6, -26}, {23.5, -28}, {28.2, -19}}));
// polys.push_back(math::Polygon2d({{7.1, -1.3}, {6.8, -3.2}, {19, -3.66}, {20.9, -1.329}}));

// obstacle.push_back({-30, 30, 0.1});
// obstacle.push_back({-15, 30, 0.8});
// obstacle.push_back({0, 30, 0.3});
// obstacle.push_back({15, 30, 0.56});
// obstacle.push_back({30, 30, 0.26});
// obstacle.push_back({-30, -30, 0.75});
// obstacle.push_back({-15, -30, 0.83});
// obstacle.push_back({0, -30, 0.34});
// obstacle.push_back({15, -30, 0.2});
// obstacle.push_back({30, -30, 0.98});
// obstacle.push_back({-30, 15, 0.25});
// obstacle.push_back({-15, 15, 0.34});
// obstacle.push_back({0, 15, 0.63});
// obstacle.push_back({15, 15, 0.45});
// obstacle.push_back({30, 15, 0.72});
// obstacle.push_back({-30, -15, 0.23});
// obstacle.push_back({-15, -15, 0.62});
// obstacle.push_back({0, -15, 0.27});
// obstacle.push_back({15, -15, 0.86});
// obstacle.push_back({30, -15, 0.56});
// obstacle.push_back({22.5, 0, 0.25});
// obstacle.push_back({-22.5, 0, 0.89});
// for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
// math::Vec2d centre(obstacle[obs_ind].x(), obstacle[obs_ind].y());
// std::vector<math::Vec2d> poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), false);
// poly_vertices_set.push_back(poly_vertices);
// polys.push_back(math::Polygon2d(poly_vertices));
// } 
// auto interactive_cb = [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg) {
// int idx = msg->marker_name.back() - '1';

// auto new_poly = polys[idx];
// new_poly.Move({ msg->pose.position.x, msg->pose.position.y });
// env->polygons().at(idx) = new_poly;
// };
// writeObstacleToYAML(poly_vertices_set, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/obs.yaml");
env->polygons() = polys;
for (int i = 4; i < polys.size(); i++) {
obs << polys[i].points()[0].x(), polys[i].points()[1].x(), polys[i].points()[2].x(), polys[i].points()[3].x(),
polys[i].points()[0].y(), polys[i].points()[1].y(), polys[i].points()[2].y(), polys[i].points()[3].y();
iris_problem.addObstacle(obs);
}
// for(int i = 0; i < polys.size(); i++) {
// auto marker = CreateMarker(i+1, polys[i], 0.2, visualization::Color::Magenta);
// server_.insert(marker, interactive_cb);
// }

// server_.applyChanges();

visualization::Init(nh, "odom", "/hmfpc_test_vis");

TrajectoryPoint start, goal;
FullStates solution, solution_car;
FullStates solution_ref, solution_car_like1, solution_car_like2, solution_diff_drive1, solution_diff_drive2, solution_diff_drive3, solution_diff_drive4;
double max_error1, max_error2;
// start.x = -28;
// start.y = 12.5;
// start.theta = M_PI / 2;
// goal.x = -10;
// goal.y = 28;
// goal.theta = 0;
// start.x = 45;
// start.y = 20.5;
// start.theta = M_PI / 2;
// goal.x = 15;
// goal.y = 15;
goal.theta = M_PI / 2;
ros::Rate r(10);

auto start_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, [&](const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose) {
double x = pose->pose.pose.position.x;
double y = pose->pose.pose.position.y;

double min_distance = DBL_MAX;
int idx = -1;
for(int i = 0; i < solution.states.size(); i++) {
double distance = hypot(solution.states[i].x - x, solution.states[i].y - y);
if(distance < min_distance) {
min_distance = distance;
idx = i;
}
}

start = solution.states[idx];
});
int count_exp = -1;
int count_factor = 0;
bool success = false;
bool solve_fail = false;
bool show_cr = false;
std::vector<std::pair<double, double>> weight_vec;
weight_vec.push_back(std::make_pair(0.0, 0.0));
weight_vec.push_back(std::make_pair(1, 1));
weight_vec.push_back(std::make_pair(0.0, 1));
weight_vec.push_back(std::make_pair(1, 0.0));
for (int i = 1; i < 32; i++) {
for (int j = 1; j < 32; j++) {
weight_vec.push_back(std::make_pair(0.05 * pow(1.1, i), 0.05 * pow(1.1, j)));
}
}

int case_index = 4;
int case_index_ = 11;
config_->opti_w_diff_drive = 0.05;
while(ros::ok()) { 
std::vector<double> new_vector;
new_vector.resize(11, 0.0);
if (success || count_exp == -1 || solve_fail) {
success = false;
solve_fail = false;
count_factor = 0;
solution.tf = 1e-31;
solution.states.clear();
solution_car.tf = 1e-31;
solution_car.states.clear();
solution_diff_drive1.tf = 1e-31;
solution_diff_drive1.states.clear();
solution_diff_drive2.tf = 1e-31;
solution_diff_drive2.states.clear();
// start = generateRandomStartPoint();
// goal = generateRandomGoalPoint();
// start.x = 1.3;
// start.y = 26;
// start.theta = -M_PI / 2;
// goal.x = 1.8;
// goal.y = -15;
// goal.theta = -M_PI / 2;
// start.x = -3.5;
// start.y = 21.300000000000001;
// start.theta = -1.5707963267948966;
// goal.x = -6.2999999999999998;
// goal.y = -15;
// goal.theta = -2.3561944901923448;
if (case_index == 0) {
start.x = -28;
start.y = 10;
start.theta = 0.0;
goal.x = 25;
goal.y = 6;
goal.theta = 0.0;
}
if (case_index == 1) {
start.x = -28;
start.y = -8;
start.theta = 0.0;
goal.x = 25;
goal.y = -8;
goal.theta = 0.0;
}

if (case_index == 2) {
start.x = -8;
start.y = -25;
start.theta = 1.5707963267948966;
goal.x = 8;
goal.y = 25;
goal.theta = 1.5707963267948966 / 2;
}

if (case_index == 3) {
start.x = -8;
start.y = 25;
start.theta = -1.5707963267948966 / 2;
goal.x = 8;
goal.y = -25;
goal.theta = -1.5707963267948966 / 2;
}

if (case_index == 4) {
// start.x = -20;
// start.y = -12;
// start.theta = 0.0;
// goal.x = 20;
// goal.y = 20;
// goal.theta = 1.5707963267948966;
std::string file_name = "/home/weijian/mir_robot_test/src/match_mobile_robotics/data"+std::to_string(case_index_) + "_1.yaml";
YAML::Node config = YAML::LoadFile(file_name);
std::vector<Data> data(config.size());
try {
// 读取 YAML 文件
// 遍历文件中的每个数据条目
for (std::size_t i = 0; i < config.size(); ++i) {
YAML::Node sublist = config[i];
if (sublist.IsSequence()) {
for (std::size_t j = 0; j < sublist.size(); ++j) {
YAML::Node dataNode = sublist[j];
if (dataNode.IsMap()) {
dataNode >> data[i];

// 输出读取的数据
} else {
throw std::runtime_error("Invalid data node");
}
}
} else {
throw std::runtime_error("Invalid sublist node");
}
}
} catch (const YAML::Exception &e) {
std::cerr << "YAML Exception: " << e.what() << std::endl;
} catch (const std::exception &e) {
std::cerr << "Exception: " << e.what() << std::endl;
}
start.x = -72;
start.y = 42;
start.theta = 0.0;
goal.x = 48;
goal.y = 41;
goal.theta = M_PI / 2;
start.x = data[0].x;
start.y = data[0].y;
start.theta = data[0].theta;
goal.x = data.back().x;
goal.y = data.back().y;
goal.theta = data.back().theta;
}
// start.x = -25 - 5 * sqrt(3) / 2;
// start.y = 5 / 2;
// start.theta = 0;
// goal.x = 25 - 5 * sqrt(3) / 2;
// goal.y = 5 / 2;
// goal.theta = 0;
// start.x = -43;
// start.y = 23;
// start.theta = 0;
// goal.x = 35;
// goal.y = 5;
// goal.theta = 0;
// config_->opti_t = 1.0;
// config_->opti_w_diff_drive = 1.0;
if (count_exp == -1) {
count_exp++;
}
}
auto traj_1 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/gazebo_demo/trl_2.yaml");
auto solution1 = traj2fullstates(traj_1);
// DrawTrajectoryRvizSE(solution1, config_, 3, path_pub_car2);

for (int i = 0; i < polys.size(); i++) {
visualization::PlotPolygon(polys[i], 0.5, visualization::Color::White, i, "Obstacle"+ std::to_string(i));
}
visualization::Trigger(); 
double coarse_path_time, solve_time_leader, solve_time_diff1, solve_time_diff2, solution_car_like;
const std::vector<heterogeneous_formation_controller::visualization::Vector> corridor_sets;
// if (count_factor > 5)
// config_->opti_w_diff_drive *= 5;
if (count_factor == 1) {
show_cr = true;
}
else {
show_cr = false;
}
// goal.x = 1.0;
// if(!planner_->Plan(solution_ref, start, goal, iris_problem, solution_ref, coarse_path_time, solve_time_leader, true, corridor_sets)) {
// solve_fail = true;
// continue;
// }
// show_cr = false;
// goal.x = 2.0;
// if(!planner_->Plan(solution_car_like1, start, goal, iris_problem, solution_car_like1, coarse_path_time, solve_time_leader, true, corridor_sets)) {
// solve_fail = true;
// continue;
// } 
// auto traj_leader = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/traj_leader.yaml");
if(!planner_->Plan(solution, start, goal, iris_problem, solution, coarse_path_time, solve_time_leader, show_cr, corridor_sets)) {
solve_fail = true;
continue;
} 
auto traj_leader = fulltotraj(solution);
double solve_time_car = 0.0;
if (!planner_->Plan_car_like(solution, 3.0, solution_car_like1, solve_time_car)) {
solve_fail = true;
continue;
}
// if (!planner_->Plan_car_like(solution, 4.0, solution_car_like2, solve_time_car)) {
// solve_fail = true;
// continue;
// }
double infeasible = 0.0;
// if (!planner_->Plan_car_like_replan(solution_ref, solution, solution, 1, infeasible, solution_car_like)) {
// solve_fail = true;
// continue;
// }
if (!planner_->Plan_car_like_replan(solution_car_like1, solution_car, solution_car, 1, infeasible, solution_car_like)) {
solve_fail = true;
continue;
}
// goal.x = 0.0;
// // auto traj_fol_diff2 = generate_ref_traj_diff(traj_leader, 5.5, 9 * M_PI / 10);
// // auto traj_fol_diff3 = generate_ref_traj_diff(traj_leader, 4.5, 3 * M_PI / 4);
// // auto traj_fol_diff4 = generate_ref_traj_diff(traj_leader, 4.0, M_PI / 3);

// // auto traj_fol_diff1 = generate_ref_traj_diff(traj_leader, 3.5, atan2(10.0-10.0, 10.0-8.0));
// // auto traj_fol_diff2 = generate_ref_traj_diff(traj_leader, hypot(3.5, 3.5), atan2(12.016733665-10, 10-8));
// // auto traj_fol_diff1 = generate_ref_traj_diff(traj_leader, 1.7*hypot(-10-(-10), -8.0-(-10.0)), atan2(10.0-10.0, 10.0-8.0));
// // auto traj_fol_diff2 = generate_ref_traj_diff(traj_leader, 1.7*hypot(-10-(-12.016733665), -10.0-(-8.0)), atan2(12.016733665-10, 10-8));
// FullStates traj_fol_diff1;
// if(!planner_->Plan(traj_fol_diff1, start, goal, iris_problem, traj_fol_diff1, coarse_path_time, solve_time_leader, true, corridor_sets)) {
// solve_fail = true;
// continue;
// } 
auto traj_fol_diff1 = generate_ref_traj_diff(traj_leader, 1.5*sqrt(5), atan2(1.5, 3));
if(!planner_->Plan_diff_drive(traj_fol_diff1, solution_diff_drive1, start, goal, solution_diff_drive1, 1, max_error1, solve_time_diff1)) {
solve_fail = true;
continue;
}
// if(!planner_->Plan_diff_drive(traj_fol_diff2, solution_diff_drive2, start, goal, solution_diff_drive2, 1, max_error1, solve_time_diff1)) {
// solve_fail = true;
// continue;
// }
// if(!planner_->Plan_diff_drive(traj_fol_diff3, solution_diff_drive3, start, goal, solution_diff_drive3, 1, max_error1, solve_time_diff1)) {
// solve_fail = true;
// continue;
// }
// if(!planner_->Plan_diff_drive(traj_fol_diff4 , solution_diff_drive4, start, goal, solution_diff_drive4, 1, max_error1, solve_time_diff1)) {
// solve_fail = true;
// continue;
// }
// if(!planner_->Plan_diff_drive(traj_fol_diff2, solution_diff_drive2, start, goal, solution_diff_drive2, 1, max_error2, solve_time_diff2)) {
// solve_fail = true;
// continue;
// }
double max_error = 0.0, avg_error = 0.0, error_time = 0.0;
for (int i = 0; i < solution_car.states.size(); i++) {
error_time = hypot(solution_car_like1.states[i].x - solution_car.states[i].x, solution_car_like1.states[i].y - solution_car.states[i].y);
if (error_time > max_error) {
max_error = error_time;
}
avg_error += error_time;
}
for (int i = 0; i < solution_diff_drive1.states.size(); i++) {
error_time = hypot(traj_fol_diff1.states[i].x - solution_diff_drive1.states[i].x, traj_fol_diff1.states[i].y - solution_diff_drive1.states[i].y);
if (error_time > max_error) {
max_error = error_time;
}
avg_error += error_time;
}
// for (int i = 0; i < solution_diff_drive2.states.size(); i++) {
// error_time = hypot(traj_fol_diff2.states[i].x - solution_diff_drive2.states[i].x, traj_fol_diff2.states[i].y - solution_diff_drive2.states[i].y);
// if (error_time > max_error) {
// max_error = error_time;
// }
// avg_error += error_time;
// }
avg_error /= 2 * (solution.states.size());
double v_avg = 0.0;
double v_max = 0.0;
double phi_avg = 0.0, phi_max = 0.0;
double a_avg = 0.0;
for (int i = 0; i < solution_car.states.size(); i++) {
if (v_max < solution_car.states[i].v) {
v_max = solution_car.states[i].v;
}
}
for (int i = 0; i < solution_car.states.size(); i++) {
if (v_max < solution_car.states[i].v) {
v_max = solution_car.states[i].v;
}
}
for (int i = 0; i < solution_diff_drive1.states.size(); i++) {
if (v_max < solution_diff_drive1.states[i].v) {
v_max = solution_diff_drive1.states[i].v;
}
}
for (int i = 0; i < solution.states.size(); i++) {
if (v_max < solution.states[i].v) {
v_max = solution.states[i].v;
}
}
for (int i = 0; i < solution_car.states.size(); i++) {
if (phi_max < solution_car.states[i].omega) {
phi_max = solution_car.states[i].omega;
}
phi_avg += fabs(solution_car.states[i].omega);
}
for (int i = 0; i < solution.states.size(); i++) {
if (phi_max < solution.states[i].omega) {
phi_max = solution.states[i].omega;
}
phi_avg += fabs(solution.states[i].omega);
}
for (int i = 0; i < solution_diff_drive1.states.size(); i++) {
if (phi_max < solution_diff_drive1.states[i].omega) {
phi_max = solution_diff_drive1.states[i].omega;
}
phi_avg += fabs(solution_diff_drive1.states[i].omega);
}
phi_avg /= (3 * solution.states.size());
double distance = 0.0;
for (int i = 1; i < solution.states.size(); i++) {
distance += hypot(solution.states[i].x - solution.states[i-1].x, solution.states[i].y - solution.states[i-1].y);
}
for (int i = 0; i < traj_fol_diff1.states.size(); i++) {
a_avg += fabs(solution_diff_drive1.states[i].a);
a_avg += fabs(solution.states[i].a);
a_avg += fabs(solution_car.states[i].a);
}
a_avg /= (3 * traj_fol_diff1.states.size());
v_avg = distance / solution.tf;
// if (!planner_->Plan_car_like(solution, 2.016733665, solution_car_like1)) {
// solve_fail = true;
// continue;
// }
// if(!planner_->Plan_diff_drive(traj_fol_diff2, start, goal, solution_diff_drive2, 1)) {
// break;
// }
// delete_yaml("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/traj_leader.yaml");
// delete_yaml("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/traj_diff1.yaml");
// // delete_yaml("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/traj_diff2.yaml");
// delete_yaml("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/traj_car1.yaml");
// if (max_error < 0.33) {
// // writeTrajectoryToYAML(solution, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/ras_demo/trl_3.yaml");
// writeTrajectoryToYAML(solution_car_like1, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/ras_demo/corridor_rec/traj_leader.yaml");

// }
// if (count_factor > 5) {
new_vector[0] = distance; 
new_vector[1] = a_avg; 
new_vector[2] = phi_max; 
new_vector[3] = v_max; 
new_vector[4] = config_->opti_w_diff_drive; 
new_vector[5] = v_avg;
new_vector[6] = config_->opti_w_err; 
new_vector[7] = phi_avg; 
new_vector[8] = solution.tf; 
new_vector[9] = max_error; 
new_vector[10] = avg_error;

// writeVectorToYAML(new_vector, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/fp_compare_2023.yaml");
// config_->opti_w_diff_drive = weight_vec[count_factor - 6].first;
// config_->opti_w_err = weight_vec[count_factor - 6].second; 
// }
if (count_factor == 30) {
  count_factor  = 0;
  case_index_++;
  success = true;
  // break;
}
// if (count_factor > 6 + weight_vec.size()) {
// success = true;
// count_exp++;
// }
if (count_exp == 200) {
break;
}
if (case_index_ == 12) {
break;
}
// config_->opti_w_diff_drive *= 1.5;
// writeTrajectoryToYAML(solution, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/gazebo_demo/trl_3.yaml");
// writeTrajectoryToYAML(solution, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/flexible_formation/traj_leader.yaml");
// writeTrajectoryToYAML(solution_diff_drive1, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/flexible_formation/traj_diff1.yaml");
// writeTrajectoryToYAML(solution_car_like1, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/flexible_formation/traj_car1.yaml");
// writeTrajectoryToYAML(solution_diff_drive2, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/ras_demo/formation_coordination/H_environment/formation1/corner_rec/traj_diff2.yaml");
// for (int i = 0; i < solution.states.size(); i++) {
// auto x0_disc = config_->vehicle.GetVertexPositions(solution.states[i].x, solution.states[i].y, solution.states[i].theta);
// std::vector<double> x1, y1;
// x1.push_back(x0_disc[0]);x1.push_back(x0_disc[2]);x1.push_back(x0_disc[4]);x1.push_back(x0_disc[6]);x1.push_back(x0_disc[0]);
// y1.push_back(x0_disc[1]);y1.push_back(x0_disc[3]);y1.push_back(x0_disc[5]);y1.push_back(x0_disc[7]);y1.push_back(x0_disc[1]);
// visualization::Plot(x1, y1, 0.2, visualization::Color::Red, i, "convex_hall");
// }
visualization::Trigger(); 
DrawTrajectoryRviz(solution, config_, 0, path_pub);
DrawTrajectoryRviz(solution_car, config_, 1, path_pub_car1);
// DrawTrajectoryRviz(solution_car_like2, config_, 2, path_pub_car2);
DrawTrajectoryRviz(solution_diff_drive1, config_, 3, path_pub_diff1);
// DrawTrajectoryRviz(solution_diff_drive2, config_, 4, path_pub_diff2);
// DrawTrajectoryRviz(solution_diff_drive3, config_, 5, path_pub_diff3);
// DrawTrajectoryRviz(solution_diff_drive4, config_, 6, path_pub_diff4);
count_factor++; 
// writeTrajectoryToYAML(solution, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/irregular_formation/result/trl_1.yaml");
// writeTrajectoryToYAML(solution_car_like1, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/irregular_formation/result/trl_2.yaml");
// writeTrajectoryToYAML(solution_car_like2, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/irregular_formation/result/trl_3.yaml");
// writeTrajectoryToYAML(solution_diff_drive1, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/irregular_formation/result/trl_4.yaml");
// writeTrajectoryToYAML(solution_diff_drive2, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/irregular_formation/result/trl_5.yaml");
// writeTrajectoryToYAML(solution_diff_drive3, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/irregular_formation/result/trl_6.yaml");
// writeTrajectoryToYAML(solution_diff_drive4, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/irregular_formation/result/trl_7.yaml");

ros::spinOnce();
r.sleep();
}

ros::spin();
return 0;
}


// /**
// * file animation.cpp
// * author Weijian Zhang (wxz163@student.bham.ac.uk)
// * brief formation planning algorithm test
// * data 2023-11-22
// * 
// * @copyright Copyroght(c) 2023
// */
// #include <ros/ros.h>
// #include <memory>
// #include "heterogeneous_formation_controller/heterogeneous_formation_controller.h"
// #include "heterogeneous_formation_controller/visualization/plot.h"

// #include <nav_msgs/Path.h>
// #include <visualization_msgs/InteractiveMarker.h>
// #include <interactive_markers/interactive_marker_server.h>
// #include <tf/transform_datatypes.h>
// #include <tf2/utils.h>

// #include "iris/iris.h"
// #include "heterogeneous_formation_controller/yaml_all.h"
// #include "heterogeneous_formation_controller/math/generate_obs.h"
// #include <Eigen/Core>
// #include <math.h>
// #include <random>
// #include <cmath>
// #include <iostream>
// #include <fstream>

// using namespace heterogeneous_formation_controller;

// // 遍历文件中的每个数据条目
// struct Data {
// double x;
// double y;
// double theta;
// };
// visualization_msgs::InteractiveMarker CreateMarker(int i, const math::Polygon2d &polygon, double width, visualization::Color c) {
// visualization_msgs::InteractiveMarker marker;
// marker.header.frame_id = "map";
// marker.header.stamp = ros::Time::now();
// marker.name = "Obstacle " + std::to_string(i);
// // marker.pose.position.x = polygon.center().x();
// // marker.pose.position.y = polygon.center().y();
// marker.pose.orientation.w = 1.0;

// visualization_msgs::Marker polygon_marker;
// polygon_marker.header.frame_id = marker.header.frame_id;
// polygon_marker.header.stamp = ros::Time();
// polygon_marker.ns = "Obstacles";
// polygon_marker.id = i;

// polygon_marker.action = visualization_msgs::Marker::ADD;
// polygon_marker.type = visualization_msgs::Marker::LINE_STRIP;
// polygon_marker.pose.orientation.w = 1.0;
// polygon_marker.scale.x = width;
// polygon_marker.color = c.toColorRGBA();

// for (size_t i = 0; i < polygon.num_points(); i++) {
// geometry_msgs::Point pt;
// pt.x = polygon.points().at(i).x();
// pt.y = polygon.points().at(i).y();
// polygon_marker.points.push_back(pt);
// }
// polygon_marker.points.push_back(polygon_marker.points.front());

// visualization_msgs::InteractiveMarkerControl box_control;
// box_control.always_visible = true;
// box_control.markers.push_back(polygon_marker);

// marker.controls.push_back(box_control);

// visualization_msgs::InteractiveMarkerControl move_control;
// move_control.name = "move_x";
// move_control.orientation.w = 0.707107f;
// move_control.orientation.x = 0;
// move_control.orientation.y = 0.707107f;
// move_control.orientation.z = 0;
// move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

// marker.controls.push_back(move_control);
// return marker;
// }

// void DrawTrajectoryRviz(const FullStates sol_traj,std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config,
// int robot_index, ros::Publisher path_pub) {
// const std::string robot_name = "footprint_" + std::to_string(robot_index);
// for(int i = 0; i < 1e3; i++) {
// visualization::Delete(i, robot_name);
// }
// visualization::Trigger();

// nav_msgs::Path msg;
// msg.header.frame_id = "map";
// msg.header.stamp = ros::Time::now();
// for(size_t i = 0; i < sol_traj.states.size(); i+=2) {
// geometry_msgs::PoseStamped pose;
// pose.header = msg.header;
// pose.pose.position.x = sol_traj.states[i].x;
// pose.pose.position.y = sol_traj.states[i].y;
// pose.pose.orientation = tf::createQuaternionMsgFromYaw(sol_traj.states[i].theta);
// msg.poses.push_back(pose);

// auto box = config->vehicle.GenerateBox(sol_traj.states[i].pose());
// auto color = robot_index > 0 ? visualization::Color::Green : visualization::Color::Red;
// // color.set_alpha(0.4);
// visualization::PlotPolygon(math::Polygon2d(box), 0.1, color, i, robot_name);
// }
// path_pub.publish(msg);
// visualization::Trigger();
// }

// double getRandomDouble(double min, double max) {
// static std::random_device rd;
// static std::mt19937 gen(rd());
// std::uniform_real_distribution<double> dis(min, max);
// return dis(gen);
// }

// // 生成在[-pi, pi]范围内的随机角度
// double getRandomAngle() {
// return getRandomDouble(0, M_PI / 2);
// }

// // 生成随机三维点
// TrajectoryPoint generateRandomStartPoint() {
// TrajectoryPoint point;
// // point.x = getRandomDouble(-20.0, -8.0);
// // point.y = getRandomDouble(-20.0, -5.0);
// point.x = -30.0;
// point.y = getRandomDouble(-30.0, 30.0);
// point.theta = getRandomAngle();
// // point.theta = 0.0;
// return point;
// }

// TrajectoryPoint generateRandomGoalPoint() {
// TrajectoryPoint point;
// // point.x = getRandomDouble(8.0, 20.0);
// // point.y = getRandomDouble(0.0, 20.0);
// point.x = getRandomDouble(-30.0, 30.0);
// point.y = 30.0;
// // point.theta = M_PI / 2;
// point.theta = getRandomAngle();
// return point;
// }

// FullStates traj2fullstates(Trajectory_temp traj) {
// FullStates solution;
// solution.states.resize(traj.size());
// solution.tf = traj[traj.size() - 1].t;
// for (int i = 0; i < traj.size(); i++) {
// solution.states[i].x = traj[i].x;
// solution.states[i].y = traj[i].y;
// solution.states[i].theta = traj[i].theta;
// solution.states[i].v = traj[i].v;
// solution.states[i].phi = traj[i].phi;
// solution.states[i].a = traj[i].a;
// solution.states[i].omega = traj[i].omega;
// }
// return solution;
// }

// void operator>>(const YAML::Node& node, Data& data) {
// if (node["x"] && node["y"] && node["theta"]) {
// data.x = node["x"].as<double>();
// data.y = node["y"].as<double>();
// data.theta = node["theta"].as<double>();
// } else {
// throw std::runtime_error("Invalid node; missing one of 'x', 'y', or 'theta'");
// }
// }

// void DrawTrajectoryRvizSE(const FullStates sol_traj, std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config,
// int robot_index, ros::Publisher path_pub) {
// for (int i = 0; i < sol_traj.states.size(); i++) {
// auto x0_disc = config->vehicle.GetVertexPositions(sol_traj.states[i].x, sol_traj.states[i].y, sol_traj.states[i].theta, 2.0);
// std::vector<double> x1, y1;
// x1.push_back(x0_disc[0]);x1.push_back(x0_disc[2]);x1.push_back(x0_disc[4]);x1.push_back(x0_disc[6]);x1.push_back(x0_disc[0]);
// y1.push_back(x0_disc[1]);y1.push_back(x0_disc[3]);y1.push_back(x0_disc[5]);y1.push_back(x0_disc[7]);y1.push_back(x0_disc[1]);
// auto color = visualization::Color::White;
// color.set_alpha(0.03);
// visualization::Plot(x1, y1, 0.2, color, i, "spatial_envelopes" + std::to_string(robot_index));
// }
// }

// int main(int argc, char **argv) {
// ros::init(argc, argv, "hmfpc_test_node");

// auto config_ = std::make_shared<PlannerConfig>();
// config_->vehicle.InitializeDiscs();

// auto env = std::make_shared<Environment>(config_);
// auto planner_ = std::make_shared<hmfpcLocalPlanner>(config_, env);

// ros::NodeHandle nh;
// ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/hmfpc_test_path", 1, false);
// ros::Publisher path_pub_diff1 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_diff1", 1, false);
// ros::Publisher path_pub_diff2 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_diff2", 1, false);
// ros::Publisher path_pub_diff3 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_diff3", 1, false);
// ros::Publisher path_pub_diff4 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_diff4", 1, false);
// ros::Publisher path_pub_car1 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_car1", 1, false);
// ros::Publisher path_pub_car2 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_car2", 1, false);

// interactive_markers::InteractiveMarkerServer server_("/hmfpc_obstacle");
// math::GenerateObstacle generateobs;
// std::vector<math::Pose> obstacle;
// std::vector<math::Polygon2d> polys, polys_orig;
// std::vector<std::vector<math::Vec2d>> poly_vertices_set;

// // double inflated_radius;
// // config_->formation.calculateMinCircle(inflated_radius, 0.0, 0.0);
// // for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
// // polys.push_back(math::Polygon2d(env->InflateObstacle(poly_vertices_set[obs_ind], inflated_radius)));
// // }
// iris::IRISProblem iris_problem(2);
// Eigen::MatrixXd obs = Eigen::MatrixXd::Zero(2, 4);
// polys.push_back(math::Polygon2d({{ -34, 45}, {-35, 33}, {-11, 32}, {-12, 44}})); 
// polys.push_back(math::Polygon2d({{26.8, 29.2}, {28.6, 23.4}, {38.6, 23.6}, {38.8, 29}}));
// polys.push_back(math::Polygon2d({{-33 + 0.3, 28.8 + 0.3}, {-33 - 0.3, 28.8 + 0.3}, {-33 - 0.3, 28.8 - 0.3}, {-33 + 0.3, 28.8 - 0.3}}));
// polys.push_back(math::Polygon2d({{-33 + 0.3, 21 + 0.3}, {-33 - 0.3, 21 + 0.3}, {-33 - 0.3, 21 - 0.3}, {-33 + 0.3, 21 - 0.3}}));
// polys.push_back(math::Polygon2d({{-19.6 + 0.3, 28 + 0.3}, {-19.6 - 0.3, 28 + 0.3}, {-19.6 - 0.3, 28 - 0.3}, {-19.6 + 0.3, 28 - 0.3}}));
// polys.push_back(math::Polygon2d({{-19.5 + 0.3, 20.5 + 0.3}, {-19.5 - 0.3, 20.5 + 0.3}, {-19.5 - 0.3, 20.5 - 0.3}, {-19.5 + 0.3, 20.5 - 0.3}}));
// polys.push_back(math::Polygon2d({{ -43.7, 52.9}, {-74.3, 52.7}, {-73.7, 45.2}, {-45.2, 45.5}}));
// polys.push_back(math::Polygon2d({{ -74, 52.2}, {-74.3, 10}, {-81.2, 10}, {-81.2, 52.5},})); 
// polys.push_back(math::Polygon2d({{ -74, 10}, {-73.6, 16.1}, {-50, 22.1}, {-50, 10.7}})); 
// polys.push_back(math::Polygon2d({{ -49.7, 10}, {-53.1, 10}, {-52.3, 0}, {-50, 0}})); 
// polys.push_back(math::Polygon2d({{ -49.7, 0}, {-49.1, -2}, {75, -2}, {75, 0}})); 
// polys.push_back(math::Polygon2d({{ -42.6, 51.3}, {-1.3, 51.1}, {-1.6, 56}, {-46.1, 56}})); 
// polys.push_back(math::Polygon2d({{ -0.65, 52.5}, {-1.3, 8.75}, {24.2, 8}, {23.6, 51.2}})); 
// polys.push_back(math::Polygon2d({{ 39.5 + 1, 20 + 1}, {39.5 - 1, 20 + 1}, {39.5 - 1, 20 - 1}, {39.5 + 1, 20 - 1}})); 
// polys.push_back(math::Polygon2d({{ 72.5, -1.5}, {76.7, -1.5}, {76.7, 50}, {72.5, 50}})); 
// polys.push_back(math::Polygon2d({{ -80.5, 50.1}, {-80.5, -54}, { -82.5, -54}, {-82.5, 50.1}})); 
// polys.push_back(math::Polygon2d({{ -80.5, 51}, {75, 51}, {75, 59}, {-80.5, 59}})); 
// polys.push_back(math::Polygon2d({{ 75, 50.1}, {75, -54}, {77, -54}, {77, 50.1}})); 
// polys.push_back(math::Polygon2d({{ 75, -54}, {75, -56}, {-80.5, -56}, {-80.5, -54}})); 
// polys.push_back(math::Polygon2d({{38.7, 35}, {38.6, 33}, {41, 33}, {42, 34.5}}));
// polys.push_back(math::Polygon2d({{56.9, 33.9}, {56.9, 32.7}, {59.4, 32.5}, {59.4, 34}}));
// polys.push_back(math::Polygon2d({{39, 21}, {39, 19.7}, {40.8, 19.7}, {41, 21}}));
// polys.push_back(math::Polygon2d({{56.9, 21}, {56.9, 19.8}, {59, 19.7}, {59, 21}}));
// // for(int i = 0; i < env_->points().size(); i++) {
// // obs << env_->points()[i].x() + 0.25, env_->points()[i].x() - 0.25, env_->points()[i].x() - 0.25, env_->points()[i].x() + 0.25,
// // env_->points()[i].y() + 0.25, env_->points()[i].y() + 0.25, env_->points()[i].y() - 0.25, env_->points()[i].y() - 0.25;
// // iris_problem.addObstacle(obs);
// // }
// // add boundary
// // obs << -52, -52, -50, -50,
// // -50, 50, 50, -50;
// // iris_problem.addObstacle(obs);
// // obs << -50, -50, 50, 50,
// // 50, 52, 52, 50;
// // iris_problem.addObstacle(obs);
// // obs << 50, 52, 52, 50,
// // 50, 50, -50, -10;
// // iris_problem.addObstacle(obs);
// // obs << -50, -50, 50, 50,
// // -50, -52, -52, -50;
// // iris_problem.addObstacle(obs);
// // std::vector<math::Polygon2d> polys = {
// // math::Polygon2d({{-3, -3}, {-3, 3}, {3, 3}, {3, -3}}),
// // };
// // polys.push_back(math::Polygon2d({{-130.5, -130}, {-130.5, 130}, {-130, 130}, {-130, -130}}));
// // polys.push_back(math::Polygon2d({{-130, 130}, {-130, 130.5}, {130, 130.5}, {130, 130}}));
// // polys.push_back(math::Polygon2d({{130, 130}, {130.5, 130}, {130.5, -130}, {130, -130}}));
// // polys.push_back(math::Polygon2d({{-130, -130}, {-130, -130.5}, {130, -130.5}, {130, -130}}));

// // polys.push_back(math::Polygon2d({{-10, -5.5}, {-10, 5.5}, {10, 5.5}, {10, -5.5}}));
// // polys.push_back(math::Polygon2d({{-30.5, -30}, {-30.5, 30}, {-30, 30}, {-30, -30}}));
// // polys.push_back(math::Polygon2d({{-30, 30}, {-30, 30.5}, {30, 30.5}, {30, 30}}));
// // polys.push_back(math::Polygon2d({{30, 30}, {30.5, 30}, {30.5, -30}, {30, -30}}));
// // polys.push_back(math::Polygon2d({{-30, -30}, {-30, -30.5}, {30, -30.5}, {30, -30}}));
// // polys.push_back(math::Polygon2d({{-10, -0.5}, {-10, 0.5}, {10, 0.5}, {10, -0.5}}));

// // obstacle.push_back({-1.5, 1.5, 0});
// // polys.push_back(math::Polygon2d({{-70, -2}, {30, -2}, {30, -30}, {-70, -30}}));

// // polys.push_back(math::Polygon2d({{-14, 15}, {-12, 15}, {-12, 17}, {-14, 17}}));
// // polys.push_back(math::Polygon2d({{-14, 10}, {-12, 10}, {-12, 12}, {-14, 12}}));
// // polys.push_back(math::Polygon2d({{-28, 10}, {-26, 10}, {-26, 12}, {-28, 12}}));
// // polys.push_back(math::Polygon2d({{-28, 15}, {-26, 15}, {-26, 17}, {-28, 17}}));

// // polys.push_back(math::Polygon2d({{15, 15}, {17, 15}, {17, 17}, {15, 17}}));
// // polys.push_back(math::Polygon2d({{26, 15}, {28, 15}, {28, 17}, {26, 17}}));
// // polys.push_back(math::Polygon2d({{15, 26}, {17, 26}, {17, 28}, {15, 28}}));
// // polys.push_back(math::Polygon2d({{26, 26}, {28, 26}, {28, 28}, {26, 28}}));

// // obstacle.push_back({0, 11.5, 0});
// // obstacle.push_back({0, -11.5, 0});
// // polys.push_back(math::Polygon2d({{-30.5, -30}, {-30.5, 30}, {-30, 30}, {-30, -30}}));
// // polys.push_back(math::Polygon2d({{-30, 30}, {-30, 30.5}, {30, 30.5}, {30, 30}}));
// // polys.push_back(math::Polygon2d({{30, 30}, {30.5, 30}, {30.5, -30}, {30, -30}}));
// // polys.push_back(math::Polygon2d({{-30, -30}, {-30, -30.5}, {30, -30.5}, {30, -30}}));
// // polys.push_back(math::Polygon2d({{-27, 25}, {-23, 20}, {-17, 21}, {-15, 24}}));
// // polys.push_back(math::Polygon2d({{-20, 14.4}, {-16, 9}, {-6, 9.6}, {-14, 16}}));
// // polys.push_back(math::Polygon2d({{-17, 0}, {-18, -6}, {-6, -7.6}, {-5.3, -1.9}}));
// // polys.push_back(math::Polygon2d({{5, -8.7}, {3.7, -12.6}, {14, -20}, {15, -10}}));
// // polys.push_back(math::Polygon2d({{5.6, 13.6}, {3.7, 7.8}, {17, 6.8}, {14.2, 9.6}}));
// // polys.push_back(math::Polygon2d({{16.3, 27.8}, {16.5, 25}, {27.3, 25}, {27.3, 27.9}}));
// // polys.push_back(math::Polygon2d({{-28.4, 3.9}, {-28.3, -3.8}, {-22.6, -4}, {-20.6, 0.5}}));
// // polys.push_back(math::Polygon2d({{-27.8, -23}, {-26.4, -28.5}, {-16.2, -28.3}, {-17.5, -24.7}}));
// // polys.push_back(math::Polygon2d({{23.2, -16.4}, {15.6, -26}, {23.5, -28}, {28.2, -19}}));
// // polys.push_back(math::Polygon2d({{7.1, -1.3}, {6.8, -3.2}, {19, -3.66}, {20.9, -1.329}}));

// // obstacle.push_back({-30, 30, 0.1});
// // obstacle.push_back({-15, 30, 0.8});
// // obstacle.push_back({0, 30, 0.3});
// // obstacle.push_back({15, 30, 0.56});
// // obstacle.push_back({30, 30, 0.26});
// // obstacle.push_back({-30, -30, 0.75});
// // obstacle.push_back({-15, -30, 0.83});
// // obstacle.push_back({0, -30, 0.34});
// // obstacle.push_back({15, -30, 0.2});
// // obstacle.push_back({30, -30, 0.98});
// // obstacle.push_back({-30, 15, 0.25});
// // obstacle.push_back({-15, 15, 0.34});
// // obstacle.push_back({0, 15, 0.63});
// // obstacle.push_back({15, 15, 0.45});
// // obstacle.push_back({30, 15, 0.72});
// // obstacle.push_back({-30, -15, 0.23});
// // obstacle.push_back({-15, -15, 0.62});
// // obstacle.push_back({0, -15, 0.27});
// // obstacle.push_back({15, -15, 0.86});
// // obstacle.push_back({30, -15, 0.56});
// // obstacle.push_back({22.5, 0, 0.25});
// // obstacle.push_back({-22.5, 0, 0.89});
// // for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
// // math::Vec2d centre(obstacle[obs_ind].x(), obstacle[obs_ind].y());
// // std::vector<math::Vec2d> poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), false);
// // poly_vertices_set.push_back(poly_vertices);
// // polys.push_back(math::Polygon2d(poly_vertices));
// // } 
// // auto interactive_cb = [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg) {
// // int idx = msg->marker_name.back() - '1';

// // auto new_poly = polys[idx];
// // new_poly.Move({ msg->pose.position.x, msg->pose.position.y });
// // env->polygons().at(idx) = new_poly;
// // };
// // writeObstacleToYAML(poly_vertices_set, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/obs.yaml");
// env->polygons() = polys;
// for (int i = 2; i < polys.size(); i++) {
// obs << polys[i].points()[0].x(), polys[i].points()[1].x(), polys[i].points()[2].x(), polys[i].points()[3].x(),
// polys[i].points()[0].y(), polys[i].points()[1].y(), polys[i].points()[2].y(), polys[i].points()[3].y();
// iris_problem.addObstacle(obs);
// }
// // for(int i = 0; i < polys.size(); i++) {
// // auto marker = CreateMarker(i+1, polys[i], 0.2, visualization::Color::Magenta);
// // server_.insert(marker, interactive_cb);
// // }

// // server_.applyChanges();

// visualization::Init(nh, "map", "/hmfpc_test_vis");

// TrajectoryPoint start, goal;
// FullStates solution, solution_ref, solution_car;
// FullStates solution_car_like1, solution_car_like2, solution_diff_drive1, solution_diff_drive2, solution_diff_drive3, solution_diff_drive4;
// double max_error1, max_error2;
// // start.x = -28;
// // start.y = 12.5;
// // start.theta = M_PI / 2;
// // goal.x = -10;
// // goal.y = 28;
// // goal.theta = 0;
// // start.x = 45;
// // start.y = 20.5;
// // start.theta = M_PI / 2;
// // goal.x = 15;
// // goal.y = 15;
// goal.theta = M_PI / 2;
// ros::Rate r(10);

// auto start_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, [&](const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose) {
// double x = pose->pose.pose.position.x;
// double y = pose->pose.pose.position.y;

// double min_distance = DBL_MAX;
// int idx = -1;
// for(int i = 0; i < solution.states.size(); i++) {
// double distance = hypot(solution.states[i].x - x, solution.states[i].y - y);
// if(distance < min_distance) {
// min_distance = distance;
// idx = i;
// }
// }

// start = solution.states[idx];
// });
// int count_exp = -1;
// int count_factor = 0;
// bool success = false;
// bool solve_fail = false;
// bool show_cr = false;
// std::vector<std::pair<double, double>> weight_vec;
// weight_vec.push_back(std::make_pair(0.0, 0.0));
// weight_vec.push_back(std::make_pair(1, 1));
// weight_vec.push_back(std::make_pair(0.0, 1));
// weight_vec.push_back(std::make_pair(1, 0.0));
// for (int i = 1; i < 32; i++) {
// for (int j = 1; j < 32; j++) {
// weight_vec.push_back(std::make_pair(0.05 * pow(1.1, i), 0.05 * pow(1.1, j)));
// }
// }

// int case_index = 4;
// config_->opti_w_diff_drive = 0.05;
// int case_index_ = 0;
// // auto traj_1 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/rviz_1.yaml");
// // auto solution1 = traj2fullstates(traj_1);
// // auto traj_2 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/rviz_2.yaml");
// // auto solution2 = traj2fullstates(traj_2);
// // auto traj_3 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/rviz_3.yaml");
// // auto solution3 = traj2fullstates(traj_3);
// while(ros::ok()) { 
// std::vector<double> new_vector;
// new_vector.resize(11, 0.0);
// if (success || count_exp == -1 || solve_fail) {
// success = false;
// solve_fail = false;
// count_factor = 0;
// solution.tf = 1e-31;
// solution.states.clear();
// solution_car.tf = 1e-31;
// solution_car.states.clear();
// solution_diff_drive1.tf = 1e-31;
// solution_diff_drive1.states.clear();
// solution_diff_drive2.tf = 1e-31;
// solution_diff_drive2.states.clear();
// // start = generateRandomStartPoint();
// // goal = generateRandomGoalPoint();
// // start.x = 1.3;
// // start.y = 26;
// // start.theta = -M_PI / 2;
// // goal.x = 1.8;
// // goal.y = -15;
// // goal.theta = -M_PI / 2;
// // start.x = -3.5;
// // start.y = 21.300000000000001;
// // start.theta = -1.5707963267948966;
// // goal.x = -6.2999999999999998;
// // goal.y = -15;
// // goal.theta = -2.3561944901923448;
// if (case_index == 0) {
// start.x = -28;
// start.y = 10;
// start.theta = 0.0;
// goal.x = 25;
// goal.y = 6;
// goal.theta = 0.0;
// }
// if (case_index == 1) {
// start.x = -28;
// start.y = -8;
// start.theta = 0.0;
// goal.x = 25;
// goal.y = -8;
// goal.theta = 0.0;
// }

// if (case_index == 2) {
// start.x = -8;
// start.y = -25;
// start.theta = 1.5707963267948966;
// goal.x = 8;
// goal.y = 25;
// goal.theta = 1.5707963267948966 / 2;
// }

// if (case_index == 3) {
// start.x = -8;
// start.y = 25;
// start.theta = -1.5707963267948966 / 2;
// goal.x = 8;
// goal.y = -25;
// goal.theta = -1.5707963267948966 / 2;
// }

// if (case_index == 4) {
// // start.x = -20;
// // start.y = -12;
// // start.theta = 0.0;
// // goal.x = 20;
// // goal.y = 20;
// // goal.theta = 1.5707963267948966;
// std::string file_name = "/home/weijian/mir_robot_test/src/match_mobile_robotics/data0.yaml";
// YAML::Node config = YAML::LoadFile(file_name);
// std::vector<Data> data(config.size());
// try {
// // 读取 YAML 文件
// // 遍历文件中的每个数据条目
// for (std::size_t i = 0; i < config.size(); ++i) {
// YAML::Node sublist = config[i];
// if (sublist.IsSequence()) {
// for (std::size_t j = 0; j < sublist.size(); ++j) {
// YAML::Node dataNode = sublist[j];
// if (dataNode.IsMap()) {
// dataNode >> data[i];

// // 输出读取的数据
// } else {
// throw std::runtime_error("Invalid data node");
// }
// }
// } else {
// throw std::runtime_error("Invalid sublist node");
// }
// }
// } catch (const YAML::Exception &e) {
// std::cerr << "YAML Exception: " << e.what() << std::endl;
// } catch (const std::exception &e) {
// std::cerr << "Exception: " << e.what() << std::endl;
// }
// start.x = -72;
// start.y = 42;
// start.theta = 0.0;
// goal.x = 48;
// goal.y = 41;
// goal.theta = M_PI / 2;
// start.x = data[0].x;
// start.y = data[0].y;
// start.theta = data[0].theta;
// goal.x = data.back().x;
// goal.y = data.back().y;
// goal.theta = data.back().theta;
// }
// // start.x = -25 - 5 * sqrt(3) / 2;
// // start.y = 5 / 2;
// // start.theta = 0;
// // goal.x = 25 - 5 * sqrt(3) / 2;
// // goal.y = 5 / 2;
// // goal.theta = 0;
// // start.x = -43;
// // start.y = 23;
// // start.theta = 0;
// // goal.x = 35;
// // goal.y = 5;
// // goal.theta = 0;
// // config_->opti_t = 1.0;
// // config_->opti_w_diff_drive = 1.0;
// if (count_exp == -1) {
// count_exp++;
// }
// }
// // DrawTrajectoryRvizSE(solution1, config_, 3, path_pub_car2);

// // for (int i = 0; i < 4; i++) {
// // visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Black, i, "Boundary"+ std::to_string(i));
// // }
// // for (int i = 4; i < polys.size(); i++) {
// // visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Black, i, "Obstacle"+ std::to_string(i));
// // }
// visualization::Trigger(); 
// double coarse_path_time, solve_time_leader, solve_time_diff1, solve_time_diff2, solution_car_like;
// const std::vector<heterogeneous_formation_controller::visualization::Vector> corridor_sets;
// // if (count_factor > 5)
// // config_->opti_w_diff_drive *= 5;
// // if (count_factor == 1) {
// // show_cr = true;
// // }
// // else {
// // show_cr = false;
// // }
// goal.x = 0.0;
// goal.y = case_index_;
// heterogeneous_formation_controller::FullStates traj_fol_diff1;
// if(!planner_->Plan(traj_fol_diff1, start, goal, iris_problem, traj_fol_diff1, coarse_path_time, solve_time_leader, true, corridor_sets)) {
// solve_fail = true;
// continue;
// } 
// goal.x = 1.0;
// goal.y = case_index_;
// if(!planner_->Plan(solution_ref, start, goal, iris_problem, solution_ref, coarse_path_time, solve_time_leader, true, corridor_sets)) {
// solve_fail = true;
// continue;
// }
// show_cr = false;
// goal.x = 2.0;
// goal.y = case_index_;
// if(!planner_->Plan(solution_car_like1, start, goal, iris_problem, solution_car_like1, coarse_path_time, solve_time_leader, true, corridor_sets)) {
// solve_fail = true;
// continue;
// } 
// // // auto traj_leader = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/traj_leader.yaml");
// // auto traj_leader = fulltotraj(solution);
// // double solve_time_car = 0.0;
// // if (!planner_->Plan_car_like(solution, 3.0, solution_car_like1, solve_time_car)) {
// // solve_fail = true;
// // continue;
// // }
// // if (!planner_->Plan_car_like(solution, 4.0, solution_car_like2, solve_time_car)) {
// // solve_fail = true;
// // continue;
// // }
// double infeasible = 0.0;
// if (!planner_->Plan_car_like_replan(solution_ref, solution, solution, 1, infeasible, solution_car_like)) {
// solve_fail = true;
// continue;
// }
// if (!planner_->Plan_car_like_replan(solution_car_like1, solution_car, solution_car, 1, infeasible, solution_car_like)) {
// solve_fail = true;
// continue;
// }
// // auto traj_fol_diff1 = generate_ref_traj_diff(traj_leader, 1.5*sqrt(2), M_PI / 4);

// // // auto traj_fol_diff2 = generate_ref_traj_diff(traj_leader, 5.5, 9 * M_PI / 10);
// // // auto traj_fol_diff3 = generate_ref_traj_diff(traj_leader, 4.5, 3 * M_PI / 4);
// // // auto traj_fol_diff4 = generate_ref_traj_diff(traj_leader, 4.0, M_PI / 3);

// // // auto traj_fol_diff1 = generate_ref_traj_diff(traj_leader, 3.5, atan2(10.0-10.0, 10.0-8.0));
// // // auto traj_fol_diff2 = generate_ref_traj_diff(traj_leader, hypot(3.5, 3.5), atan2(12.016733665-10, 10-8));
// // // auto traj_fol_diff1 = generate_ref_traj_diff(traj_leader, 1.7*hypot(-10-(-10), -8.0-(-10.0)), atan2(10.0-10.0, 10.0-8.0));
// // // auto traj_fol_diff2 = generate_ref_traj_diff(traj_leader, 1.7*hypot(-10-(-12.016733665), -10.0-(-8.0)), atan2(12.016733665-10, 10-8));
// if(!planner_->Plan_diff_drive(traj_fol_diff1, solution_diff_drive1, start, goal, solution_diff_drive1, 1, max_error1, solve_time_diff1)) {
// solve_fail = true;
// continue;
// }
// // auto traj_1 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/gazebo_demo/trl_2.yaml");
// // auto solution1 = traj2fullstates(traj_1);
// // if(!planner_->Plan_diff_drive(traj_fol_diff2, solution_diff_drive2, start, goal, solution_diff_drive2, 1, max_error1, solve_time_diff1)) {
// // solve_fail = true;
// // continue;
// // }
// // if(!planner_->Plan_diff_drive(traj_fol_diff3, solution_diff_drive3, start, goal, solution_diff_drive3, 1, max_error1, solve_time_diff1)) {
// // solve_fail = true;
// // continue;
// // }
// // if(!planner_->Plan_diff_drive(traj_fol_diff4 , solution_diff_drive4, start, goal, solution_diff_drive4, 1, max_error1, solve_time_diff1)) {
// // solve_fail = true;
// // continue;
// // }
// // if(!planner_->Plan_diff_drive(traj_fol_diff2, solution_diff_drive2, start, goal, solution_diff_drive2, 1, max_error2, solve_time_diff2)) {
// // solve_fail = true;
// // continue;
// // }
// double max_error = 0.0, avg_error = 0.0, error_time = 0.0;
// for (int i = 0; i < solution_car.states.size(); i++) {
// error_time = hypot(solution_car.states[i].x - solution.states[i].x, solution_car.states[i].y - solution.states[i].y);
// error_time  = fabs(error_time - 3);
// if (error_time > max_error) {
// max_error = error_time;
// }
// avg_error += error_time;
// }
// for (int i = 0; i < solution_diff_drive1.states.size(); i++) {
// error_time = hypot(solution.states[i].x - solution_diff_drive1.states[i].x, solution.states[i].y - solution_diff_drive1.states[i].y);
// error_time  = fabs(error_time - 1.5*sqrt(5));
// if (error_time > max_error) {
// max_error = error_time;
// }
// avg_error += error_time;
// }
// for (int i = 0; i < solution_diff_drive1.states.size(); i++) {
// error_time = hypot(solution_car.states[i].x - solution_diff_drive1.states[i].x, solution_car.states[i].y - solution_diff_drive1.states[i].y);
// error_time  = fabs(error_time - 1.5*sqrt(5));
// if (error_time > max_error) {
// max_error = error_time;
// }
// avg_error += error_time;
// }
// avg_error /= (3 * solution.states.size());
// double v_avg = 0.0;
// double v_max = 0.0;
// double phi_avg = 0.0, phi_max = 0.0;
// double a_avg = 0.0;
// for (int i = 0; i < solution_car.states.size(); i++) {
// if (v_max < solution_car.states[i].v) {
// v_max = solution_car.states[i].v;
// }
// }
// for (int i = 0; i < solution_diff_drive1.states.size(); i++) {
// if (v_max < solution_diff_drive1.states[i].v) {
// v_max = solution_diff_drive1.states[i].v;
// }
// }
// // for (int i = 0; i < solution_diff_drive2.states.size(); i++) {
// // if (v_max < solution_diff_drive2.states[i].v) {
// // v_max = solution_diff_drive2.states[i].v;
// // }
// // }
// for (int i = 0; i < solution.states.size(); i++) {
// if (v_max < solution.states[i].v) {
// v_max = solution.states[i].v;
// }
// }
// for (int i = 0; i < solution_car.states.size(); i++) {
// if (phi_max < solution_car.states[i].omega) {
// phi_max = solution_car.states[i].omega;
// }
// phi_avg += fabs(solution_car.states[i].omega);
// }
// for (int i = 0; i < solution.states.size(); i++) {
// if (phi_max < solution.states[i].omega) {
// phi_max = solution.states[i].omega;
// }
// phi_avg += fabs(solution.states[i].omega);
// }
// for (int i = 0; i < solution_diff_drive1.states.size(); i++) {
// phi_avg += fabs(solution_diff_drive1.states[i].omega);
// }
// phi_avg /= (3 * solution_diff_drive1.states.size());
// double distance = 0.0;
// for (int i = 1; i < solution.states.size(); i++) {
// distance += hypot(solution.states[i].x - solution.states[i-1].x, solution.states[i].y - solution.states[i-1].y);
// }
// for (int i = 0; i < solution_diff_drive1.states.size(); i++) {
// a_avg += fabs(solution_diff_drive1.states[i].a);
// a_avg += fabs(solution.states[i].a);
// a_avg += fabs(solution_car.states[i].a);
// }
// a_avg /= (3 * solution_diff_drive1.states.size());
// v_avg = distance / solution.tf;
// // if (!planner_->Plan_car_like(solution, 2.016733665, solution_car_like1)) {
// // solve_fail = true;
// // continue;
// // }
// // if(!planner_->Plan_diff_drive(traj_fol_diff2, start, goal, solution_diff_drive2, 1)) {
// // break;
// // }
// // delete_yaml("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/traj_leader.yaml");
// // delete_yaml("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/traj_diff1.yaml");
// // // delete_yaml("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/traj_diff2.yaml");
// // delete_yaml("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/traj_car1.yaml");
// // if (max_error < 0.33) {
// // // writeTrajectoryToYAML(solution, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/ras_demo/trl_3.yaml");
// // writeTrajectoryToYAML(solution_car_like1, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/ras_demo/corridor_rec/traj_leader.yaml");

// // }
// // if (count_factor > 5) {
// // new_vector[0] = distance; 
// // new_vector[1] = a_avg; 
// // new_vector[2] = solve_time_leader; 
// // new_vector[3] = solution_car_like + solve_time_diff1 + solve_time_diff2; 
// // new_vector[4] = config_->opti_w_diff_drive; 
// // new_vector[5] = v_avg;
// // new_vector[6] = config_->opti_w_err; 
// // new_vector[7] = phi_avg; 
// // new_vector[8] = solution.tf; 
// // new_vector[9] = max_error; 
// // new_vector[10] = avg_error;
// // writeVectorToYAML(new_vector, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/fp_compare_2023.yaml");
// // config_->opti_w_diff_drive = weight_vec[count_factor - 6].first;
// // config_->opti_w_err = weight_vec[count_factor - 6].second; 
// // }
// success = true;
// // case_index_++;
// if (count_exp == 200) {
// break;
// }
// // if (case_index_ == 11) {
// // break;
// // }
// // config_->opti_w_diff_drive *= 1.5;
// // writeTrajectoryToYAML(solution, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/gazebo_demo/trl_3.yaml");
// // writeTrajectoryToYAML(solution, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/flexible_formation/traj_leader.yaml");
// // writeTrajectoryToYAML(solution_diff_drive1, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/flexible_formation/traj_diff1.yaml");
// // writeTrajectoryToYAML(solution_car_like1, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/flexible_formation/traj_car1.yaml");
// // writeTrajectoryToYAML(solution_diff_drive2, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/ras_demo/formation_coordination/H_environment/formation1/corner_rec/traj_diff2.yaml");
// // for (int i = 0; i < solution.states.size(); i++) {
// // auto x0_disc = config_->vehicle.GetVertexPositions(solution.states[i].x, solution.states[i].y, solution.states[i].theta);
// // std::vector<double> x1, y1;
// // x1.push_back(x0_disc[0]);x1.push_back(x0_disc[2]);x1.push_back(x0_disc[4]);x1.push_back(x0_disc[6]);x1.push_back(x0_disc[0]);
// // y1.push_back(x0_disc[1]);y1.push_back(x0_disc[3]);y1.push_back(x0_disc[5]);y1.push_back(x0_disc[7]);y1.push_back(x0_disc[1]);
// // visualization::Plot(x1, y1, 0.2, visualization::Color::Red, i, "convex_hall");
// // }
// visualization::Trigger(); 
// DrawTrajectoryRviz(solution, config_, 1, path_pub);
// DrawTrajectoryRviz(solution_car, config_, 3, path_pub_car1);
// // DrawTrajectoryRviz(solution_car_like2, config_, 2, path_pub_car2);
// DrawTrajectoryRviz(solution_diff_drive1, config_, 0, path_pub_diff1);
// // DrawTrajectoryRviz(solution_diff_drive2, config_, 4, path_pub_diff2);
// // DrawTrajectoryRviz(solution_diff_drive3, config_, 5, path_pub_diff3);
// // DrawTrajectoryRviz(solution_diff_drive4, config_, 6, path_pub_diff4);
// // writeTrajectoryToYAML(solution, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/rviz_1.yaml");
// // writeTrajectoryToYAML(solution_car, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/rviz_2.yaml");
// // writeTrajectoryToYAML(solution_diff_drive1, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/rviz_3.yaml");
// count_factor++; 
// // writeTrajectoryToYAML(solution, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/irregular_formation/result/trl_1.yaml");
// // writeTrajectoryToYAML(solution_car_like1, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/irregular_formation/result/trl_2.yaml");
// // writeTrajectoryToYAML(solution_car_like2, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/irregular_formation/result/trl_3.yaml");
// // writeTrajectoryToYAML(solution_diff_drive1, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/irregular_formation/result/trl_4.yaml");
// // writeTrajectoryToYAML(solution_diff_drive2, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/irregular_formation/result/trl_5.yaml");
// // writeTrajectoryToYAML(solution_diff_drive3, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/irregular_formation/result/trl_6.yaml");
// // writeTrajectoryToYAML(solution_diff_drive4, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/irregular_formation/result/trl_7.yaml");

// ros::spinOnce();
// r.sleep();
// }

// ros::spin();
// return 0;
// }


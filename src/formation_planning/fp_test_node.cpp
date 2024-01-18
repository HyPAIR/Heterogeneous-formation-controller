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
#include "formation_planner/formation_planner.h"
#include "formation_planner/visualization/plot.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>

#include "iris/iris.h"
#include <Eigen/Core>
#include "formation_planner/yaml_all.h"
#include "formation_planner/math/generate_obs.h"
#include <Eigen/Core>
#include <math.h>
#include <random>
#include <cmath>

using namespace formation_planner;

visualization_msgs::InteractiveMarker CreateMarker(int i, const math::Polygon2d &polygon, double width, visualization::Color c) {
  visualization_msgs::InteractiveMarker marker;
  marker.header.frame_id = "map";
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

void DrawTrajectoryRviz(const FullStates sol_traj,std::shared_ptr<formation_planner::PlannerConfig> config,
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "fp_test_node");

  auto config_ = std::make_shared<PlannerConfig>();
  config_->vehicle.InitializeDiscs();

  auto env = std::make_shared<Environment>(config_);
  auto planner_ = std::make_shared<FormationPlanner>(config_, env);

  ros::NodeHandle nh;
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/fp_test_path", 1, false);
  ros::Publisher path_pub_diff1 = nh.advertise<nav_msgs::Path>("/fp_test_path_diff1", 1, false);
  ros::Publisher path_pub_diff2 = nh.advertise<nav_msgs::Path>("/fp_test_path_diff2", 1, false);
  ros::Publisher path_pub_car1 = nh.advertise<nav_msgs::Path>("/fp_test_path_car1", 1, false);

  interactive_markers::InteractiveMarkerServer server_("/fp_obstacle");
  math::GenerateObstacle generateobs;
  std::vector<math::Pose> obstacle;
  std::vector<math::Polygon2d> polys, polys_orig;
  std::vector<std::vector<math::Vec2d>> poly_vertices_set;

  // double inflated_radius;
  // config_->formation.calculateMinCircle(inflated_radius, 0.0, 0.0);
  // for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
  //   polys.push_back(math::Polygon2d(env->InflateObstacle(poly_vertices_set[obs_ind], inflated_radius)));
  // }
  iris::IRISProblem iris_problem(2);
  Eigen::MatrixXd obs = Eigen::MatrixXd::Zero(2, 4);
  // polys.push_back(math::Polygon2d({{-33 + 0.3, 28.8 + 0.3}, {-33 - 0.3, 28.8 + 0.3}, {-33 - 0.3, 28.8 - 0.3}, {-33 + 0.3, 28.8 - 0.3}}));
  // polys.push_back(math::Polygon2d({{-33 + 0.3, 21 + 0.3}, {-33 - 0.3, 21 + 0.3}, {-33 - 0.3, 21 - 0.3}, {-33 + 0.3, 21 - 0.3}}));
  // polys.push_back(math::Polygon2d({{-20 + 0.3, 28.8 + 0.3}, {-20 - 0.3, 28.8 + 0.3}, {-20 - 0.3, 28.8 - 0.3}, {-20 + 0.3, 28.8 - 0.3}}));
  // polys.push_back(math::Polygon2d({{-20 + 0.3, 21 + 0.3}, {-20 - 0.3, 21 + 0.3}, {-20 - 0.3, 21 - 0.3}, {-20 + 0.3, 21 - 0.3}}));
  // polys.push_back(math::Polygon2d({{ -43.7, 52.9}, {-74.3, 52.7}, {-73.7, 45.2}, {-45.2, 45.5}}));
  // polys.push_back(math::Polygon2d({{ -74, 52.2}, {-81.2, 52.5}, {-74.3, 10}, {-81.2, 10}})); 
  // polys.push_back(math::Polygon2d({{ -74, 10}, {-73.6, 16.1}, {-51.3, 22.1}, {-51, 10.7}})); 
  // polys.push_back(math::Polygon2d({{ -49.7, 10}, {-53.1, 10}, {-52.3, 0}, {-51, 0}})); 
  // polys.push_back(math::Polygon2d({{ -49.7, 0}, {-49.1,  -2}, {75, -2}, {75, 0}})); 
  // polys.push_back(math::Polygon2d({{ -42.6, 51.3}, {-1.3,  51.1}, {-1.6, 56}, {-46.1,  56}})); 
  // polys.push_back(math::Polygon2d({{ -0.65, 52.5}, {-1.3,  8.75}, {24.2, 8}, {23.6,  51.2}})); 
  // polys.push_back(math::Polygon2d({{ 39.5 + 1, 20 + 1}, {39.5 - 1,  20 + 1}, {39.5 - 1, 20 - 1}, {39.5 + 1,  20 - 1}})); 
  // polys.push_back(math::Polygon2d({{ 72.5, -1.5}, {76.7,  -1.5}, {76.7, 50}, {72.5,  50}})); 
  // polys.push_back(math::Polygon2d({{ -80.5, 57}, {-80.5,  -54}, { -82.5, -54}, {-82.5,  57}})); 
  // polys.push_back(math::Polygon2d({{ -80.5, 57}, {75,  57}, {75, 59}, {-80.5,  59}})); 
  // polys.push_back(math::Polygon2d({{ 75, 57}, {75,  -54}, {77, -54}, {77,  57}})); 
  // polys.push_back(math::Polygon2d({{ 75, -54}, {75,  -56}, {-80.5, -56}, {-80.5, -54}})); 

  // for(int i = 0; i < env_->points().size(); i++) {
  //   obs << env_->points()[i].x() + 0.25, env_->points()[i].x() - 0.25, env_->points()[i].x() - 0.25, env_->points()[i].x() + 0.25,
  //          env_->points()[i].y() + 0.25, env_->points()[i].y() + 0.25, env_->points()[i].y() - 0.25, env_->points()[i].y() - 0.25;
    // iris_problem.addObstacle(obs);
  // }
  // add boundary
  // obs << -52, -52, -50, -50,
  //       -50, 50, 50, -50;
  // iris_problem.addObstacle(obs);
  // obs << -50, -50, 50, 50,
  //       50, 52, 52, 50;
  // iris_problem.addObstacle(obs);
  // obs << 50, 52, 52, 50,
  //       50, 50, -50, -10;
  // iris_problem.addObstacle(obs);
  // obs << -50, -50, 50, 50,
  //       -50, -52, -52, -50;
  // iris_problem.addObstacle(obs);
  // std::vector<math::Polygon2d> polys = {
  //     math::Polygon2d({{-3, -3}, {-3, 3}, {3, 3}, {3, -3}}),
  // };
  // polys.push_back(math::Polygon2d({{-130.5, -130}, {-130.5, 130}, {-130, 130}, {-130, -130}}));
  // polys.push_back(math::Polygon2d({{-130, 130}, {-130, 130.5}, {130, 130.5}, {130, 130}}));
  // polys.push_back(math::Polygon2d({{130, 130}, {130.5, 130}, {130.5, -130}, {130, -130}}));
  // polys.push_back(math::Polygon2d({{-130, -130}, {-130, -130.5}, {130, -130.5}, {130, -130}}));

  polys.push_back(math::Polygon2d({{-30.5, -30}, {-30.5, 30}, {-30, 30}, {-30, -30}}));
  polys.push_back(math::Polygon2d({{-30, 30}, {-30, 30.5}, {30, 30.5}, {30, 30}}));
  polys.push_back(math::Polygon2d({{30, 30}, {30.5, 30}, {30.5, -30}, {30, -30}}));
  polys.push_back(math::Polygon2d({{-30, -30}, {-30, -30.5}, {30, -30.5}, {30, -30}}));
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

  obstacle.push_back({0, 18.5, 0});
  obstacle.push_back({0, -18.5, 0});
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
  for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
    math::Vec2d centre(obstacle[obs_ind].x(), obstacle[obs_ind].y());
    std::vector<math::Vec2d> poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), false);
    poly_vertices_set.push_back(poly_vertices);
    polys.push_back(math::Polygon2d(poly_vertices));
  } 
  // auto interactive_cb = [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg) {
  //   int idx = msg->marker_name.back() - '1';

  //   auto new_poly = polys[idx];
  //   new_poly.Move({ msg->pose.position.x, msg->pose.position.y });
  //   env->polygons().at(idx) = new_poly;
  // };
  // writeObstacleToYAML(poly_vertices_set, "/home/weijian/Heterogeneous_formation/src/formation_planner/traj_result/obs.yaml");
  env->polygons() = polys;
  for (int i = 0; i < polys.size(); i++) {
    obs << polys[i].points()[0].x(), polys[i].points()[1].x(), polys[i].points()[2].x(), polys[i].points()[3].x(),
            polys[i].points()[0].y(), polys[i].points()[1].y(), polys[i].points()[2].y(), polys[i].points()[3].y();
    iris_problem.addObstacle(obs);
  }
  // for(int i = 0; i < polys.size(); i++) {
  //   auto marker = CreateMarker(i+1, polys[i], 0.2, visualization::Color::Magenta);
  //   server_.insert(marker, interactive_cb);
  // }

  // server_.applyChanges();

  visualization::Init(nh, "odom", "/fp_test_vis");

  TrajectoryPoint start, goal;
  FullStates solution, solution_car;
  FullStates solution_car_like1, solution_diff_drive1, solution_diff_drive2;
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
      // start.x = -18;
      // start.y = -14;
      // start.theta = 1.5707963267948966;
      // goal.x = 12;
      // goal.y = 10;
      // goal.theta = 1.5707963267948966;
      start.x = -16;
      start.y = -10;
      start.theta = M_PI / 2;
      goal.x = 12;
      goal.y = 10;
      goal.theta = M_PI / 2;
      // start.x = -43;
      // start.y = 23;
      // start.theta = 0;
      // goal.x = 35;
      // goal.y = 5;
      // goal.theta = 0;
      config_->opti_t = 1.0;
      config_->opti_w_diff_drive = 0.05;
      if (count_exp == -1) {
        count_exp++;
      }
    }
    for (int i = 0; i < 4; i++) {
      visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Black, i, "Boundary"+  std::to_string(i));
    }
    for (int i = 0; i < polys.size(); i++) {
      visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Black, i, "Obstacle"+  std::to_string(i));
    }
    visualization::Trigger();   
    double coarse_path_time, solve_time_leader, solve_time_diff1, solve_time_diff2, solution_car_like;

    if(!planner_->Plan(solution, start, goal, iris_problem, solution, coarse_path_time, solve_time_leader, show_cr)) {
      solve_fail = true;
      continue;
    }
    show_cr = false;
    // auto traj_leader = generate_traj("/home/weijian/Heterogeneous_formation/src/formation_planner/traj_result/traj_leader.yaml");
    auto traj_leader = fulltotraj(solution);
    double solve_time_car = 0.0;
    if (!planner_->Plan_car_like(solution, 1.7*2.016733665, solution_car_like1, solve_time_car)) {
      solve_fail = true;
      continue;
    }
    double infeasible = 0.0;
    // if (!planner_->Plan_car_like_replan(solution_car_like1, solution_car, solution_car, 1, infeasible, solution_car_like)) {
    //   solve_fail = true;
    //   continue;
    // }
    auto traj_fol_diff1 = generate_ref_traj_diff(traj_leader, 1.7*hypot(-18-(-20), -21.0083668325-(-20)), atan2(21.0083668325-20, 20-18));
    // auto traj_fol_diff1 = generate_ref_traj_diff(traj_leader, 1.7*hypot(-10-(-10), -8.0-(-10.0)), atan2(10.0-10.0, 10.0-8.0));
    // auto traj_fol_diff2 = generate_ref_traj_diff(traj_leader, 1.7*hypot(-10-(-12.016733665), -10.0-(-8.0)), atan2(12.016733665-10, 10-8));
    if(!planner_->Plan_diff_drive(traj_fol_diff1, solution_diff_drive1, start, goal, solution_diff_drive1, 1, max_error1, solve_time_diff1)) {
      solve_fail = true;
      continue;
    }
    // if(!planner_->Plan_diff_drive(traj_fol_diff2, solution_diff_drive2, start, goal, solution_diff_drive2, 1, max_error2, solve_time_diff2)) {
    //   solve_fail = true;
    //   continue;
    // }
    double max_error = 0.0, avg_error = 0.0, error_time = 0.0;
    // for (int i = 0; i < solution_car.states.size(); i++) {
    //   error_time = hypot(solution_car_like1.states[i].x - solution_car.states[i].x, solution_car_like1.states[i].y - solution_car.states[i].y);
    //   if (error_time > max_error) {
    //     max_error = error_time;
    //   }
    //   avg_error += error_time;
    // }
    for (int i = 0; i < solution_diff_drive1.states.size(); i++) {
      error_time = hypot(traj_fol_diff1.states[i].x - solution_diff_drive1.states[i].x, traj_fol_diff1.states[i].y - solution_diff_drive1.states[i].y);
      if (error_time > max_error) {
        max_error = error_time;
      }
      avg_error += error_time;
    }
    // for (int i = 0; i < solution_diff_drive2.states.size(); i++) {
    //   error_time = hypot(traj_fol_diff2.states[i].x - solution_diff_drive2.states[i].x, traj_fol_diff2.states[i].y - solution_diff_drive2.states[i].y);
    //   if (error_time > max_error) {
    //     max_error = error_time;
    //   }
    //   avg_error += error_time;
    // }
    avg_error /= (3*solution.states.size());
    double v_avg = 0.0;
    double v_max = 0.0;
    double phi_avg = 0.0, phi_max = 0.0;
    for (int i = 0; i < solution_car.states.size(); i++) {
      if (v_max < solution_car.states[i].v) {
        v_max = solution_car.states[i].v;
      }
    }
    // for (int i = 0; i < solution_diff_drive1.states.size(); i++) {
    //   if (v_max < solution_diff_drive1.states[i].v) {
    //     v_max = solution_diff_drive1.states[i].v;
    //   }
    // }
    // for (int i = 0; i < solution_diff_drive2.states.size(); i++) {
    //   if (v_max < solution_diff_drive2.states[i].v) {
    //     v_max = solution_diff_drive2.states[i].v;
    //   }
    // }
    for (int i = 0; i < solution.states.size(); i++) {
      if (v_max < solution.states[i].v) {
        v_max = solution.states[i].v;
      }
    }
    for (int i = 0; i < solution_car.states.size(); i++) {
      if (phi_max < solution_car.states[i].phi) {
        phi_max = solution_car.states[i].phi;
      }
      phi_avg += fabs(solution_car.states[i].phi);
    }
    for (int i = 0; i < solution.states.size(); i++) {
      if (phi_max < solution.states[i].phi) {
        phi_max = solution.states[i].phi;
      }
      phi_avg += fabs(solution.states[i].phi);
    }
    phi_avg /= (2 * solution.states.size());
    double distance = 0.0;
    for (int i = 1; i < solution.states.size(); i++) {
      distance += hypot(solution.states[i].x - solution.states[i-1].x, solution.states[i].y - solution.states[i-1].y);
    }
    v_avg = distance / solution.tf;
    // if (!planner_->Plan_car_like(solution, 2.016733665, solution_car_like1)) {
    //   solve_fail = true;
    //   continue;
    // }
    // if(!planner_->Plan_diff_drive(traj_fol_diff2, start, goal, solution_diff_drive2, 1)) {
    //   break;
    // }
    // delete_yaml("/home/weijian/Heterogeneous_formation/src/formation_planner/traj_result/traj_leader.yaml");
    // delete_yaml("/home/weijian/Heterogeneous_formation/src/formation_planner/traj_result/traj_diff1.yaml");
    // // delete_yaml("/home/weijian/Heterogeneous_formation/src/formation_planner/traj_result/traj_diff2.yaml");
    // delete_yaml("/home/weijian/Heterogeneous_formation/src/formation_planner/traj_result/traj_car1.yaml");
    // if (max_error < 0.33) {
    //   // writeTrajectoryToYAML(solution, "/home/weijian/Heterogeneous_formation/src/formation_planner/traj_result/ras_demo/trl_3.yaml");
    //   writeTrajectoryToYAML(solution_car_like1, "/home/weijian/Heterogeneous_formation/src/formation_planner/traj_result/ras_demo/corridor_rec/traj_leader.yaml");

    // }
    if (count_factor > 20) {
      new_vector[0] = distance; new_vector[1] = coarse_path_time; new_vector[2] = solve_time_leader; new_vector[3] = solution_car_like + solve_time_diff1 + solve_time_diff2; new_vector[4] = v_max; new_vector[5] = v_avg;
      new_vector[6] = phi_max; new_vector[7] = phi_avg; new_vector[8] = solution.tf; new_vector[9] = max_error; new_vector[10] = avg_error;
      writeVectorToYAML(new_vector, "/home/weijian/Heterogeneous_formation/src/formation_planner/traj_result/experiment_4.yaml");
      success = true;
      count_exp++;
    }
    if (count_exp == 200) {
      break;
    }
    writeTrajectoryToYAML(solution, "/home/weijian/Heterogeneous_formation/src/formation_planner/traj_result/ras_demo/formation_coordination/H_environment/formation1/traj_leader.yaml");
    writeTrajectoryToYAML(solution_diff_drive1, "/home/weijian/Heterogeneous_formation/src/formation_planner/traj_result/ras_demo/formation_coordination/H_environment/formation1/traj_diff1.yaml");
    writeTrajectoryToYAML(solution_car_like1, "/home/weijian/Heterogeneous_formation/src/formation_planner/traj_result/ras_demo/formation_coordination/H_environment/formation1/traj_car1.yaml");
    writeTrajectoryToYAML(solution_diff_drive2, "/home/weijian/Heterogeneous_formation/src/formation_planner/traj_result/ras_demo/formation_coordination/H_environment/formation1/corner_rec/traj_diff2.yaml");
    // for (int i = 0; i < solution.states.size(); i++) {
    // auto x0_disc = config_->vehicle.GetVertexPositions(solution.states[i].x, solution.states[i].y, solution.states[i].theta);
    //   std::vector<double> x1, y1;
    //   x1.push_back(x0_disc[0]);x1.push_back(x0_disc[2]);x1.push_back(x0_disc[4]);x1.push_back(x0_disc[6]);x1.push_back(x0_disc[0]);
    //   y1.push_back(x0_disc[1]);y1.push_back(x0_disc[3]);y1.push_back(x0_disc[5]);y1.push_back(x0_disc[7]);y1.push_back(x0_disc[1]);
    //   visualization::Plot(x1, y1, 0.2, visualization::Color::Red, i, "convex_hall");
    // }
    visualization::Trigger();   
    DrawTrajectoryRviz(solution, config_, 0, path_pub);
    DrawTrajectoryRviz(solution_car_like1, config_, 1, path_pub_car1);
    DrawTrajectoryRviz(solution_diff_drive1, config_, 2, path_pub_diff1);
    // DrawTrajectoryRviz(solution_diff_drive2, config_, 3, path_pub_diff2);
    count_factor++; 
    ros::spinOnce();
    r.sleep();
  }

  ros::spin();
  return 0;
}
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
#include "heterogeneous_formation_controller/topo_prm.h"
#include "heterogeneous_formation_controller/IdentifyHomotopy.h"
#include "heterogeneous_formation_controller/yaml_all.h"
#include "heterogeneous_formation_controller/math/generate_obs.h"
#include <Eigen/Core>
#include <math.h>
#include <random>
#include <cmath>

using namespace heterogeneous_formation_controller;
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

void DrawTrajectoryRviz(const FullStates sol_traj,std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config,
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

void generateRegularPolygon(const double cx, const double cy, const double r, const int k, 
  std::vector<Eigen::Vector2d>& vertice_set, std::vector<TrajectoryPoint>& tp_set) {
    std::vector<std::pair<double, double>> vertices;
    double angleIncrement = 2 * M_PI / k;

    for (int i = 0; i < k; ++i) {
        TrajectoryPoint tp_temp;
        double angle = i * angleIncrement;
        double x = cx + r * std::cos(angle);
        double y = cy + r * std::sin(angle);
        tp_temp.x = x;
        tp_temp.y = y;
        tp_temp.theta = 0.0;
        vertice_set.push_back({x, y});
        tp_set.push_back(tp_temp);
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

vector<Eigen::Vector2d> discretizePath(const vector<Eigen::Vector2d>& path, int pt_num) {
  vector<double> len_list;
  len_list.push_back(0.0);

  for (int i = 0; i < path.size() - 1; ++i) {
    double inc_l = (path[i + 1] - path[i]).norm();
    len_list.push_back(inc_l + len_list[i]);
  }

  // calc pt_num points along the path
  double len_total = len_list.back();
  double dl = len_total / double(pt_num - 1);
  double cur_l;

  vector<Eigen::Vector2d> dis_path;
  for (int i = 0; i < pt_num; ++i) {
    cur_l = double(i) * dl;

    // find the range cur_l in
    int idx = -1;
    for (int j = 0; j < len_list.size() - 1; ++j) {
      if (cur_l >= len_list[j] - 1e-4 && cur_l <= len_list[j + 1] + 1e-4) {
        idx = j;
        break;
      }
    }

    // find lambda and interpolate
    double lambda = (cur_l - len_list[idx]) / (len_list[idx + 1] - len_list[idx]);
    Eigen::Vector2d inter_pt = (1 - lambda) * path[idx] + lambda * path[idx + 1];
    dis_path.push_back(inter_pt);
  }

  return dis_path;
}

int main(int argc, char **argv) {
  vector<heterogeneous_formation_controller::visualization::Color> colors = {
    visualization::Color::Red,
    visualization::Color::Green,
    visualization::Color::Blue,
    visualization::Color::Cyan,
    visualization::Color::Yellow
  };
  ros::init(argc, argv, "hmfpc_test_node");

  auto config_ = std::make_shared<PlannerConfig>();
  config_->vehicle.InitializeDiscs();

  auto env = std::make_shared<Environment>(config_);
  auto planner_ = std::make_shared<hmfpcLocalPlanner>(config_, env);

  ros::NodeHandle nh;
  int num_robot = 4;
  std::vector<ros::Publisher> path_pub_set;
  for (int i = 0; i < num_robot; i++) {
    ros::Publisher path_pub_temp = nh.advertise<nav_msgs::Path>("/hmfpc_test_path" + std::to_string(i), 1, false);
    path_pub_set.push_back(path_pub_temp);
  }

  interactive_markers::InteractiveMarkerServer server_("/hmfpc_obstacle");
  math::GenerateObstacle generateobs;
  std::vector<math::Pose> obstacle;
  std::vector<math::Polygon2d> polys, polys_orig, polys_inflat;
  std::vector<double> height;
  std::vector<std::vector<math::Vec2d>> poly_vertices_set;

  // double inflated_radius;
  // config_->formation.calculateMinCircle(inflated_radius, 0.0, 0.0);
  // for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
  //   polys.push_back(math::Polygon2d(env->InflateObstacle(poly_vertices_set[obs_ind], inflated_radius)));
  // }
  iris::IRISProblem iris_problem(2);
  Eigen::MatrixXd obs = Eigen::MatrixXd::Zero(2, 4);
  obstacle.push_back({0, 9.5, 0});
  obstacle.push_back({0, -9.5, 0});
	obstacle.push_back({18.5, 0, 0.25});
	obstacle.push_back({-18.5, 0, 0.3});

  for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
    std::vector<math::Vec2d> poly_vertices;
    math::Vec2d centre(obstacle[obs_ind].x(), obstacle[obs_ind].y());
    if (obs_ind < 2) {
      poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), true);
    }
    else {
      poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), false);
    }
    poly_vertices_set.push_back(poly_vertices);
    polys.push_back(math::Polygon2d(poly_vertices));
  }
  for (int i = 0; i < polys.size(); i++)
  {
    height.push_back(getRandomDouble(0.2, 0.8));
  }
  
  env->heights() = height; 
  // auto interactive_cb = [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg) {
  //   int idx = msg->marker_name.back() - '1';

  //   auto new_poly = polys[idx];
  //   new_poly.Move({ msg->pose.position.x, msg->pose.position.y });
  //   env->polygons().at(idx) = new_poly;
  // };
  // writeObstacleToYAML(poly_vertices_set, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/obs.yaml");
  for (int i = 0; i < polys.size(); i++) {
    polys_inflat.push_back(math::Polygon2d({
      {polys[i].points()[0].x() - 0.5, polys[i].points()[0].y() + 0.5}, 
      {polys[i].points()[1].x() - 0.5, polys[i].points()[1].y() - 0.5}, 
      {polys[i].points()[2].x() + 0.5, polys[i].points()[2].y() - 0.5}, 
      {polys[i].points()[3].x() + 0.5, polys[i].points()[3].y() + 0.5}
      }));
  }
  env->polygons() = polys_inflat;
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

  visualization::Init(nh, "odom", "/hmfpc_test_vis");
  VVCM vvcm;
  TrajectoryPoint start, start1, start2, goal, goal1, goal2;
  FullStates solution, solution_car;
  FullStates solution_car_like1, solution_diff_drive1, solution_diff_drive2;
  double max_error1, max_error2;
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
  double start_point_x = -28;
  double start_point_y = 0;
  double goal_point_x = 28;
  double goal_point_y = 0;  
  std::vector<Eigen::Vector2d> start_pts;
  std::vector<Eigen::Vector2d> goal_pts;
  std::vector<TrajectoryPoint> start_set;
  std::vector<TrajectoryPoint> goal_set;
  generateRegularPolygon(start_point_x, start_point_y, vvcm.formation_radius, num_robot, start_pts, start_set);
  generateRegularPolygon(goal_point_x, goal_point_y, vvcm.formation_radius, num_robot, goal_pts, goal_set);
  heterogeneous_formation_controller::TopologyPRM topo_prm(config_, env);
  CoarsePathPlanner coarse_topo_path(config_, env);
  std::vector<std::vector<std::vector<double>>> corridors_sets;
  std::vector<FullStates> solution_set(num_robot);
  while(ros::ok()) { 
  int path_index = 0;
  std::vector<std::vector<heterogeneous_formation_controller::math::Pose>> topo_paths;
  std::vector<heterogeneous_formation_controller::FullStates> solution_sets;
  vector<vector<vector<Eigen::Vector2d>>> raw_paths_set;
    topo_prm.init(nh);
    list<GraphNode::Ptr>            graph;
    vector<vector<Eigen::Vector2d>> raw_paths, filtered_paths, select_paths;
    for (int i = 0; i < 4; i++) {
      visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Black, i, "Boundary"+  std::to_string(i));
    }
    for (int i = 0; i < polys.size(); i++) {
      visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Black, i, "Obstacle"+  std::to_string(i));
    }
    visualization::Trigger();   
    for (int i = 0; i < num_robot; i++) {
      topo_prm.findTopoPaths(start_pts[i], goal_pts[i], graph,
                              raw_paths, filtered_paths, select_paths);
      raw_paths_set.push_back(raw_paths);
    }
    // for (int ind = 0; ind < raw_paths_set.size(); ind++) {
    //   for (int i = 0; i < raw_paths_set[ind].size(); i++) {
    //     vector<double> x_raw(raw_paths_set[ind][i].size()), y_raw(raw_paths_set[ind][i].size());
    //     for (int j  = 0; j < raw_paths_set[ind][i].size(); j++) {
    //       x_raw[j] = raw_paths_set[ind][i][j].x();
    //       y_raw[j] = raw_paths_set[ind][i][j].y();
    //     }
    //     colors[ind].set_alpha(0.4);
    //     visualization::Plot(x_raw, y_raw, 0.5, colors[ind], 1, "Raw Path" + std::to_string(path_index++));
    //     visualization::Trigger();
    //   }
    // }
    env->polygons() = polys;
    auto corridors_sets = CalCorridors(raw_paths_set, env);
    double coarse_path_time, solve_time_leader;
    show_cr = false;
    for (int i = 0; i < corridors_sets.size(); i++) {
      for (int ind_c = 0; ind_c < 3; ind_c++) {
        solution_set[ind_c].states.clear();
      }
      // if (raw_paths.size() >= 3) {
        // if(!planner_->Plan(solution, start_set[2], goal_set[2], iris_problem, solution, coarse_path_time, solve_time_leader, show_cr, corridors_sets[2])) {
        //   ROS_ERROR("re-plan trajectory optimization failed!");
        // }
      if(!planner_->Plan_fm(
        solution_set, start_set, goal_set, iris_problem, solution_set, coarse_path_time, 
        solve_time_leader, show_cr, corridors_sets[i], path_pub_set)) {
        ROS_ERROR("re-plan trajectory optimization failed!");
        continue;
      }
      else {
        solution_sets.push_back(solution);   
      }
      for (int j = 0; j < solution_set.size(); j++) {
        DrawTrajectoryRviz(solution_set[j], config_, j, path_pub_set[j]);
      }
      for (int j = 0; j < solution_set.size(); j++) {
        writeTrajectoryToYAML(solution_set[j], "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/flexible_formation/traj_" + std::to_string(num_robot) + std::to_string(j) + ".yaml");
      }        
      // }
      std::vector<std::vector<double>> topo_(3);
      std::vector<std::vector<double>> relative_(3);
      for (int i = 0; i < solution_set[0].states.size(); i++) {
        topo_[0].push_back((solution_set[0].states[i].x - solution_set[2].states[i].x) * (solution_set[2].states[i].y - solution_set[1].states[i].y) +
                          (solution_set[0].states[i].y - solution_set[2].states[i].y) * (solution_set[1].states[i].x - solution_set[2].states[i].x));
        topo_[1].push_back((solution_set[1].states[i].x - solution_set[0].states[i].x) * (solution_set[0].states[i].y - solution_set[2].states[i].y) +
                          (solution_set[1].states[i].y - solution_set[0].states[i].y) * (solution_set[2].states[i].x - solution_set[0].states[i].x));
        topo_[2].push_back((solution_set[2].states[i].x - solution_set[1].states[i].x) * (solution_set[1].states[i].y - solution_set[0].states[i].y) +
                          (solution_set[2].states[i].y - solution_set[1].states[i].y) * (solution_set[0].states[i].x - solution_set[1].states[i].x));
      }
      for (int i = 0; i < solution_set[0].states.size(); i++) {
        relative_[0].push_back(sqrt(pow(solution_set[1].states[i].x - solution_set[0].states[i].x, 2) + pow(solution_set[1].states[i].y - solution_set[0].states[i].y, 2)));
        relative_[1].push_back(sqrt(pow(solution_set[2].states[i].x - solution_set[1].states[i].x, 2) + pow(solution_set[2].states[i].y - solution_set[1].states[i].y, 2)));
        relative_[2].push_back(sqrt(pow(solution_set[0].states[i].x - solution_set[2].states[i].x, 2) + pow(solution_set[0].states[i].y - solution_set[2].states[i].y, 2)));
      }
      // }
    }
    // vector<vector<double>> x_dis(20), y_dis(20);
    // if (raw_paths_dis_set.size() == 3) {
    //   for (int i = 0; i < 20; i++) {
    //     for (int j = 0; j < raw_paths_dis_set.size(); j++) {
    //       x_dis[i].push_back(raw_paths_dis_set[j][i].x());
    //       y_dis[i].push_back(raw_paths_dis_set[j][i].y());
    //     }
    //     auto color = visualization::Color::Grey;
    //     color.set_alpha(1);
    //     visualization::PlotPolygon(x_dis[i], y_dis[i], 0.5, color, 1, "Formation_dis" + std::to_string(i));
    //     visualization::Trigger();
    //   }
    // }
    // vector<vector<double>> xs(solution_sets.size()), ys(solution_sets.size());
    // for (int i = 0; i < solution_sets.size(); i++) {
    //   for (int j = 0; j < solution_sets[i].states.size(); j++) {
    //     xs[i].push_back(solution_sets[i].states[j].x);
    //     ys[i].push_back(solution_sets[i].states[j].y);
    //   }
    //   colors[i].set_alpha(1.0);
    //   visualization::Plot(xs[i], ys[i], 0.5, colors[i], 1, "Coarse Path" + std::to_string(i));
    //   visualization::Trigger();
    // }
    // const double PI = 3.14159265358979323846;
    // std::random_device rd;
    // std::mt19937 gen(rd());
    // std::uniform_real_distribution<> dis(0, 2 * PI);

    // // 生成三个随机角度
    // double angle1 = dis(gen);
    // double angle2 = dis(gen);
    // double angle3 = dis(gen);
    // double y3 = sin(angle3);
    // VVCM::Point p1(1.2 / sqrt(3) * cos(angle1), 1.2 / sqrt(3) * sin(angle1)), p2(1.2 / sqrt(3) * cos(angle2), 1.2 / sqrt(3) * sin(angle2)), p3(1.2 / sqrt(3) * cos(angle3), 1.2 / sqrt(3) * sin(angle3)), pvo, ro;
    // double zo;
    // vvcm.cak_direct_kinmatics(p1, p2, p3, pvo, ro, zo);
    // auto color = visualization::Color::Magenta;
    // std::vector<double> x = {p1.x, p2.x, p3.x, p1.x};
    // std::vector<double> y = {p1.y, p2.y, p3.y, p1.y};
    // std::vector<double> xo = {ro.x+0.1, ro.x+0.1, ro.x-0.1, ro.x-0.1, ro.x+0.1};
    // std::vector<double> yo = {ro.y+0.1, ro.y-0.1, ro.y-0.1, ro.y+0.1, ro.y+0.1};
    // visualization::Plot(x, y, 0.05, color, 1, "point_s");
    // visualization::Trigger();
    // color = visualization::Color::Red;
    // visualization::Plot(xo, yo, 0.01, color, 1, "point_ro");
    // visualization::Trigger();5
    // ROS_WARN("zo: %f", zo);
    ros::spinOnce();
    r.sleep();
  }

  ros::spin();
  return 0;
}
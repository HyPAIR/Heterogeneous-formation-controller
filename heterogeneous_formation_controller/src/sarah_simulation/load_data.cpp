#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>
#include <visualization_msgs/InteractiveMarker.h>
#include "heterogeneous_formation_controller/heterogeneous_formation_controller.h"
#include "heterogeneous_formation_controller/math/generate_obs.h"
#include "heterogeneous_formation_controller/visualization/plot.h"
#include "heterogeneous_formation_controller/math/math_utils.h"
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/ScopedState.h>
#include "yaml-cpp/yaml.h"
#include "heterogeneous_formation_controller/yaml_all.h"
#include "std_msgs/Int32.h"

using json = nlohmann::json;
using namespace heterogeneous_formation_controller;
namespace ob = ompl::base;
int traj_ind = 0;
int time_step = 0;
struct MotionPrimitive {
    std::string name;
    double turningRadius;
    std::vector<std::string> motionTypes;
    std::vector<double> motionLengths;
};
template<class T, class U>
std::vector<math::Pose> InterpolatePath(
    const std::shared_ptr<T> &space,
    U &path,
    const ob::ScopedState<> &start,
    const ob::ScopedState<> &goal,
    double step_size);

void writetime_count(const std::vector<int> &time_set, std::string filename) {
    YAML::Emitter emitter;
    emitter << YAML::BeginSeq;
    for (const auto& element : time_set) {
        emitter << element;
    }
    emitter << YAML::EndSeq;
    std::ofstream fout(filename);
    fout << emitter.c_str();
    fout.close();

    std::cout << "YAML file written successfully.\n";
}

void readtime_count(std::vector<int> &time_set, std::string filename) {
  try {
    // 从文件中加载 YAML 文档
    YAML::Node config = YAML::LoadFile(filename);

    // 确保文档是一个序列
    if (config.IsSequence()) {
        // 读取序列中的每个元素并存储到 std::vector<int> 中
        time_set = config.as<std::vector<int>>();
    } else {
        std::cerr << "Error: YAML document is not a sequence.\n";
    }
  } catch (const YAML::Exception& e) {
      std::cerr << "YAML Exception: " << e.what() << "\n";
  }
}

void PlotPose(std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config, int robot_index, int time_step, Trajectory_temp traj) {
    const std::string robot_name = "Footprint_" + std::to_string(robot_index);
    // visualization::Delete(i, robot_name);
    // visualization::Trigger();
    double theta = traj[time_step].theta;
    if (theta < -M_PI) {
        while (theta < -M_PI) {
            theta += 2 * M_PI;
        }
    }
    else if (theta > M_PI) {
        while (theta > M_PI) {
            theta -= 2 * M_PI;
        }
    }
    math::Pose robot_pos(traj[time_step].x, traj[time_step].y, theta);
    auto box = config->vehicle.GenerateBox(robot_pos);
    auto color = visualization::Color::Magenta;
    visualization::PlotPolygon(math::Polygon2d(box), 0.1, color, 1, robot_name);
    visualization::Trigger();
    // visualization::Plot(xs, ys, 0.5, visualization::Color::Magenta, i, robot_name_ta);
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

void DrawPathRivz(const std::vector<std::vector<double>> path, int robot_index) {
  std::vector<double> xs, ys;
  const std::string robot_name = "Footprint_" + std::to_string(robot_index);
  for (int i = 0; i < path.size(); i++) {
    xs.push_back(path[i][0]);
    ys.push_back(path[i][1]);
  }
  auto color = visualization::Color::White;
  color.set_alpha(0.4);
  visualization::Plot(xs, ys, 0.3, color, robot_index, robot_name);
  visualization::Trigger();
}

void DrawCircle(const double& x, const double& y, const double radius, const int& wp_index, const int obs_ind) {
    std::vector<double> xwp, ywp;
    const std::string robot_name = "Waypoint" + std::to_string(wp_index);
    for (int i = 0; i < 100; i++) {
        double theta = 2 * M_PI / 100 * i;
        xwp.push_back(x + radius * cos(theta));
        ywp.push_back(y + radius * sin(theta));
    }
    auto color = obs_ind == 1 ? visualization::Color::Red : visualization::Color::Blue;
    color.set_alpha(0.7);
    visualization::PlotPoints(xwp, ywp, 0.15, color, wp_index, robot_name);
    visualization::Trigger();
}

bool GenerateShortestPath(math::Pose start, math::Pose goal, std::vector<math::Pose> &result, const double& turing_radius) {
  double step_size = 0.1;
  std::shared_ptr<ob::SE2StateSpace> state_space;
  bool is_forward_only_ = true;
  if(is_forward_only_) {
    state_space = std::make_shared<ob::DubinsStateSpace>(turing_radius);
  } else {
    state_space = std::make_shared<ob::ReedsSheppStateSpace>(turing_radius);
  }

  ob::ScopedState<ob::SE2StateSpace> rs_start(state_space), rs_goal(state_space);
  rs_start[0] = start.x();
  rs_start[1] = start.y();
  rs_start[2] = start.theta();
  rs_goal[0] = goal.x();
  rs_goal[1] = goal.y();
  rs_goal[2] = goal.theta();

  if(is_forward_only_) {
    auto ss = std::static_pointer_cast<ob::DubinsStateSpace>(state_space);
    auto path = ss->dubins(rs_start.get(), rs_goal.get());
    result = InterpolatePath(ss, path, rs_start, rs_goal, step_size);
  } else {
    auto ss = std::static_pointer_cast<ob::ReedsSheppStateSpace>(state_space);
    auto path = ss->reedsShepp(rs_start.get(), rs_goal.get());
    result = InterpolatePath(ss, path, rs_start, rs_goal, step_size);
  }

  return true;
}

template<class T, class U>
std::vector<math::Pose> InterpolatePath(
    const std::shared_ptr<T> &space,
    U &path,
    const ob::ScopedState<> &start,
    const ob::ScopedState<> &goal,
    double step_size) {
  std::vector<math::Pose> result;
  ob::ScopedState<> state(space);

  bool first_time = false;
  int sample_count = std::max(1, static_cast<int>(ceil(path.length() / step_size)));

  result.resize(sample_count+1);
  for(int i = 0; i < sample_count+1; i++) {
    space->interpolate(start.get(), goal.get(), static_cast<double>(i) / sample_count, first_time, path, state.get());
    result[i].setX(state[0]);
    result[i].setY(state[1]);
    result[i].setTheta(state[2]);
  }
  return result;
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
    auto color = robot_index > 0 ? visualization::Color::Yellow : visualization::Color::White;
    color.set_alpha(0.1);
    visualization::PlotPolygon(math::Polygon2d(box), 0.05, color, i, robot_name);
  }
  path_pub.publish(msg);
  visualization::Trigger();
}

FullStates generateTrajectory(const MotionPrimitive& primitive, double startX, double startY, double startTheta, int totalPoints) {
    FullStates trajectory;

    // 初始状态
    double currentX = startX;
    double currentY = startY;
    double currentTheta = startTheta;
    currentTheta = 0.0;

    for (size_t i = 0; i < primitive.motionTypes.size(); ++i) {
        std::string motionType = primitive.motionTypes[i];
        double motionLength = primitive.motionLengths[i];

        // 设置每一步产生的轨迹点数
        int pointsPerStep = totalPoints / primitive.motionTypes.size();

        // 根据运动类型进行逐步移动
        for (int point = 0; point < pointsPerStep; ++point) {
            double ratio = static_cast<double>(point) / static_cast<double>(pointsPerStep);
            double endX, endY, endTheta;

            double angle = motionLength * ratio / primitive.turningRadius;

            if (motionType == "L") {
                // 左转
                endX = currentX + primitive.turningRadius * std::sin(angle + currentTheta);
                endY = currentY - primitive.turningRadius * (1 - std::cos(angle + currentTheta));
                endTheta = currentTheta + angle;
            } else if (motionType == "R") {
                // 右转
                endX = currentX - primitive.turningRadius * std::sin(angle - currentTheta);
                endY = currentY - primitive.turningRadius * (1 - std::cos(angle - currentTheta));
                endTheta = currentTheta - angle;
            }

            // 插值计算轨迹点
            double interpolatedX = currentX + ratio * (endX - currentX);
            double interpolatedY = currentY + ratio * (endY - currentY);
            TrajectoryPoint temp_pt;
            temp_pt.x = interpolatedX;
            temp_pt.y = interpolatedY;
            temp_pt.theta = endTheta;
            trajectory.states.push_back(temp_pt);

            // 更新当前状态
            currentX = interpolatedX;
            currentY = interpolatedY;
            // currentTheta = endTheta;
        }
    }

    return trajectory;
}

std::vector<double> GenerateOptimalTimeProfileSegment(const std::vector<double> &stations, double start_time, std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config_) {
  double max_accel = config_->vehicle.max_acceleration; double max_decel = -config_->vehicle.max_acceleration;
  double max_velocity = config_->vehicle.max_velocity; double min_velocity = -config_->vehicle.max_velocity;

  int accel_idx = 0, decel_idx = stations.size()-1;
  double vi = 0.0;
  std::vector<double> profile(stations.size());
  for (int i = 0; i < stations.size()-1; i++) {
    double ds = stations[i+1] - stations[i];

    profile[i] = vi;
    vi = sqrt(vi * vi + 2 * max_accel * ds);
    vi = std::min(max_velocity, std::max(min_velocity, vi));

    if(vi >= max_velocity) {
      accel_idx = i+1;
      break;
    }
  }

  vi = 0.0;
  for (int i = stations.size()-1; i > accel_idx; i--) {
    double ds = stations[i] - stations[i-1];
    profile[i] = vi;
    vi = sqrt(vi * vi - 2 * max_decel * ds);
    vi = std::min(max_velocity, std::max(min_velocity, vi));

    if(vi >= max_velocity) {
      decel_idx = i;
      break;
    }
  }

  std::fill(std::next(profile.begin(), accel_idx), std::next(profile.begin(), decel_idx), max_velocity);

  std::vector<double> time_profile(stations.size(), start_time);
  for(size_t i = 1; i < stations.size(); i++) {
    if(profile[i] < 1e-6) {
      time_profile[i] = time_profile[i-1];
      continue;
    }
    time_profile[i] = time_profile[i-1] + (stations[i] - stations[i-1]) / profile[i];
  }
  return time_profile;
}

FullStates ResamplePath(const std::vector<math::Pose> &path, std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config_) {
  std::vector<int> gears(path.size());
  std::vector<double> stations(path.size(), 0);

  for(size_t i = 1; i < path.size(); i++) {
    double tracking_angle = atan2(path[i].y() - path[i-1].y(), path[i].x() - path[i-1].x());
    bool gear = std::abs(math::NormalizeAngle(tracking_angle - path[i].theta())) < M_PI_2;
    gears[i] = gear ? 1 : -1;

    stations[i] = stations[i-1] + path[i].DistanceTo(path[i-1]);
  }

  if(gears.size() > 1) {
    gears[0] = gears[1];
  }

  std::vector<double> time_profile(gears.size());
  size_t last_idx = 0;
  double start_time = 0;
  for(size_t i = 0; i < gears.size(); i++) {
    if(i == gears.size() - 1 || gears[i+1] != gears[i]) {
      std::vector<double> station_segment;
      std::copy_n(stations.begin(), i - last_idx + 1, std::back_inserter(station_segment));

      auto profile = GenerateOptimalTimeProfileSegment(station_segment, start_time, config_);
      std::copy(profile.begin(), profile.end(), std::next(time_profile.begin(), last_idx));
      start_time = profile.back();
      last_idx = i;
    }
  }

  // int nfe = std::max(config_->min_nfe, int(time_profile.back() / config_->time_step));
  int nfe = path.size();
  auto interpolated_ticks = math::LinSpaced(time_profile.front(), time_profile.back(), nfe);

  std::vector<double> prev_x(path.size()), prev_y(path.size()), prev_theta(path.size());
  for(size_t i = 0; i < path.size(); i++) {
    prev_x[i] = path[i].x();
    prev_y[i] = path[i].y();
    prev_theta[i] = path[i].theta();
  }

  FullStates result;
  result.tf = interpolated_ticks.back();
  result.states.resize(nfe);
  auto interp_x = math::Interpolate1d(time_profile, prev_x, interpolated_ticks);
  auto interp_y = math::Interpolate1d(time_profile, prev_y, interpolated_ticks);
  auto interp_theta = math::ToContinuousAngle(math::Interpolate1d(time_profile, prev_theta, interpolated_ticks));
  for(size_t i = 0; i < nfe; i++) {
    result.states[i].x = interp_x[i];
    result.states[i].y = interp_y[i];
    result.states[i].theta = interp_theta[i];
  }

  double dt = interpolated_ticks[1] - interpolated_ticks[0];
  for(size_t i = 0; i < nfe-1; i++) {
    double tracking_angle = atan2(result.states[i+1].y - result.states[i].y, result.states[i+1].x - result.states[i].x);
    bool gear = std::abs(math::NormalizeAngle(tracking_angle - result.states[i].theta)) < M_PI_2;
    double velocity = hypot(result.states[i+1].y - result.states[i].y, result.states[i+1].x - result.states[i].x) / dt;

    result.states[i].v = std::min(config_->vehicle.max_velocity, std::max(-config_->vehicle.max_velocity, gear ? velocity : -velocity));
    result.states[i].phi = std::min(config_->vehicle.phi_max, std::max(-config_->vehicle.phi_min, atan((result.states[i+1].theta - result.states[i].theta) * config_->vehicle.wheel_base / (result.states[i].v * dt))));
  }

  for(size_t i = 0; i < nfe-1; i++) {
    result.states[i].a = std::min(config_->vehicle.max_acceleration, std::max(-config_->vehicle.max_acceleration, (result.states[i+1].v - result.states[i].v) / dt));
    result.states[i].omega = std::min(config_->vehicle.omega_max, std::max(-config_->vehicle.omega_max, (result.states[i+1].phi - result.states[i].phi) / dt));
  }
  return result;
}

void integerCallback(const std_msgs::Int32::ConstPtr& msg) {
    traj_ind = msg->data;
    // ROS_INFO("Received integer: %d", msg->data);
}

int main(int argc, char* argv[]) {
    double scale = 3;
    double step_size = 0.1;
    std::ifstream file("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/data.json");

    if (!file.is_open()) {
        std::cerr << "Failed to open JSON file." << std::endl;
        return 1;
    }

    json jsonData;
    file >> jsonData;

    file.close();

    std::vector<std::vector<double>> complete_path;
    std::vector<math::Pose> complete_path_;
    std::vector<std::vector<std::string>> motionTypes_set;
    std::vector<std::vector<double>> motionLengths_set;
    std::vector<std::string> name_set;
    std::vector<std::vector<math::Pose>> traj_set;
    double obstacleRadius = jsonData["ObstacleRadius"];
    double turningRadius = jsonData["TurningRadius"];
    turningRadius *= scale;

    std::map<int, std::array<double, 2>> waypointsMap;
    for (const auto& waypoint : jsonData["Waypoints"].items()) {
        int waypointKey = std::stoi(waypoint.key());
        waypointsMap[waypointKey] = waypoint.value();
    }

    std::cout << "Sorted Waypoints:\n";
    for (const auto& waypoint : waypointsMap) {
        std::cout << "Waypoint " << waypoint.first << ": (" << waypoint.second[0] << ", " << waypoint.second[1] << ")\n";
    }

    json motionPrimitives = jsonData["MotionPrimitves"];
    for (const auto& primitive : motionPrimitives) {
        std::string name = primitive["name"];
        double turningRadius = primitive["turningRadius"];
        std::vector<std::string> motionTypes = primitive["motionTypes"];
        std::vector<double> motionLengths = primitive["motionLengths"];
        for (int i = 0; i < motionLengths.size(); i++) {
          motionLengths[i] = scale * motionLengths[i];
        }
        std::cout << "Name: " << name << "\n";
        std::cout << "Turning Radius: " << turningRadius << "\n";
        std::cout << "Motion Types: ";
        for (const auto& type : motionTypes) {
            std::cout << type << " ";
        }
        std::cout << "\n";
        std::cout << "Motion Lengths: ";
        for (const auto& length : motionLengths) {
            std::cout << length << " ";
        }
        std::cout << "\n\n";
        motionTypes_set.push_back(motionTypes);
        motionLengths_set.push_back(motionLengths);
        name_set.push_back(name);
    }

    json sequence = jsonData["Sequence"];
    for (const auto& step : sequence) {
        complete_path.push_back({scale * waypointsMap[step][0], scale * waypointsMap[step][1]});
    }
    complete_path_.resize(complete_path.size());
    for (int i = 1; i < complete_path.size(); i++) {
      math::Pose pose_temp(complete_path[i][0], complete_path[i][1], atan2(complete_path[i][1] - complete_path[i-1][1], 
        complete_path[i][0] - complete_path[i-1][0]));
        complete_path_[i] = pose_temp;
    }
    for (int i = 1; i < complete_path_.size(); i++) {
      if (complete_path_[i].theta() - complete_path_[i-1].theta() > M_PI) {
        complete_path_[i-1].setTheta( complete_path_[i-1].theta() +  2 * M_PI);
      }
    }
    MotionPrimitive primitive;
    primitive.name = "60 degree left turn";
    primitive.turningRadius = turningRadius;
    primitive.motionTypes = motionTypes_set[2];
    primitive.motionLengths = motionLengths_set[2];

    // 生成轨迹，起点坐标为 (0, 0)
    FullStates trajectory = generateTrajectory(primitive, complete_path_[1].x(), complete_path_[1].y(), complete_path_[1].theta(), 200);
    math::Pose pose_temp(complete_path[0][0], complete_path[0][1], complete_path_[1].theta());
    complete_path_[0] = pose_temp;
    for (int i = 0; i < complete_path_.size() - 1; i++) {
      std::vector<math::Pose> result_1;
      std::vector<math::Pose> result_2;
      math::Pose pose_temp(complete_path_[i].x(), complete_path_[i].y(), complete_path_[i+1].theta());
      if(GenerateShortestPath(complete_path_[i], pose_temp, result_1, turningRadius))
      ROS_WARN("Traj_%d turning motion successfully generates!", i);
      if(GenerateShortestPath(pose_temp, complete_path_[i+1], result_2, turningRadius))
      ROS_WARN("Traj_%d straight motion successfully generates!", i);
      traj_set.push_back(result_1);
      traj_set.push_back(result_2);
    }
    for (int i = 0; i < traj_set.size(); i += 2) {
      if (traj_set[i][0].theta() == -3.1415926535897931)
      {
        traj_set[i][0].setTheta(M_PI);
      }
    }
    ros::init(argc, argv, "hmfpc_test_node");
    auto config_ = std::make_shared<PlannerConfig>();
    config_->vehicle.InitializeDiscs();
    auto env = std::make_shared<Environment>(config_);
    auto planner_ = std::make_shared<hmfpcLocalPlanner>(config_, env);
    ros::NodeHandle nh;
    visualization::Init(nh, "map", "/hmfpc_test_vis");
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/hmfpc_test_path", 1, false);\
    ros::Subscriber integer_sub = nh.subscribe("integer_topic", 10, integerCallback);
    ros::Rate rate(40);
    TrajectoryPoint start, goal;
    std::vector<FullStates> ref_traj_set;
    std::vector<FullStates> solution_set;
    FullStates ref_traj, sol_all;
    double max_error, infeasibility = inf;
    for (int i = 0; i < traj_set.size(); i++) {
      ref_traj = ResamplePath(traj_set[i], config_);
      for (int j = 0; j < ref_traj.states.size(); j++) {
        if (j != ref_traj.states.size() - 1 && fabs(ref_traj.states[j+1].theta - ref_traj.states[j].theta) > 0.3) {
          ref_traj.states[j+1].theta = M_PI;
          if (ref_traj.states[j].theta * ref_traj.states[j+2].theta < 0) {
            for (int k = j + 1; k < ref_traj.states.size(); k++) {
              if (ref_traj.states[k].theta < 0) {
                ref_traj.states[k].theta += 2 * M_PI;
              }
            }
          }
          break;
        }
      }
      if (ref_traj.tf != 0) {
        ref_traj_set.push_back(ref_traj);
      }
      // for (int k = 0; k < 5; k++) {
      //   ref_traj.states.push_back(ref_traj.states.back());
      // }
    }
    double distance = 0.0;
    for (int i = 0; i < ref_traj.states.size(); i++) {
      distance += hypot(ref_traj.states[i].x - ref_traj.states[i-1].x, ref_traj.states[i].y - ref_traj.states[i-1].y);
    }
    ref_traj.tf = step_size * (ref_traj.states.size() - 1);
    start.x = complete_path_[0].x();
    start.y = complete_path_[0].y();
    start.theta = complete_path_[0].theta();
    goal.x = complete_path_[complete_path.size() - 1].x();
    goal.y = complete_path_[complete_path.size() - 1].y();
    goal.theta = complete_path_[complete_path.size() - 1].theta();
    FullStates solution;
    std::vector<Trajectory_temp> traj_all_set;
    Trajectory_temp traj_all;
    std::vector<int> time_set;
    std::vector<int> obs_ind;
    std::vector<bool> is_straight;
    obs_ind.resize(complete_path_.size(), 0);
    traj_all = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/sarah_traj.yaml");
    readtime_count(time_set, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/sarah_time.yaml");
    traj_all_set.resize(time_set.size());
    is_straight.resize(time_set.size(), false);
    std::vector<FullStates> solution_all;
    solution_all.resize(time_set.size());
    std::vector<int> obs_trigger;
    for (int i = 0; i < solution_all.size(); i++) {
      int time_ind_start = 0;
      int time_ind_end = 0;
      for (int j = 0; j < i; j++) {
        time_ind_start += time_set[j];
      }
      for (int j = 0; j < i+1; j++) {
        time_ind_end += time_set[j];
      }
      for (int k = time_ind_start; k < time_ind_end; k++) {
        traj_all_set[i].push_back(traj_all[k]);
      }
    }
    for (int i = 0; i < is_straight.size(); i++) {
      if (time_set[i] == 21) {
        is_straight[i] = true;
        int k = 0;
        for (int j = 0; j < i; j++) {
          k += time_set[j];
        } 
      obs_trigger.push_back(k);
    }
    }
    int j = 0;
    int obs_ind_ = 0;
    while (ros::ok()) {
      DrawPathRivz(complete_path, 0);
      for (int i = 0; i < complete_path.size(); i++) {
          DrawCircle(complete_path[i][0], complete_path[i][1], scale * obstacleRadius, i, obs_ind[i]);
      }
      // DrawTrajectoryRviz(traj2fullstates(traj_all_set[traj_ind]), config_, 1, path_pub);
      // PlotPose(config_, 1, j, traj_all);
      j = traj_ind;
      if (j > obs_trigger[obs_ind_]) {
        if (is_straight[traj_ind]) {
          obs_ind[obs_ind_++] = 1;
        }
        traj_ind++;
      }
      // j++;
      if (j == traj_all.size() - traj_all_set[traj_all.size() - 1].size()) {
        obs_ind[obs_ind_] = 1;
      }
      if (j == traj_all.size()) {
        break;
      }
      // for (int i = 0; i < ref_traj_set.size(); i++) {
      //   config_->opti_w_a = 0.1;
      //   infeasibility = inf;
      //   FullStates solution;
      //   while (infeasibility > 1e-5) {
      //     // planner_->Plan_car_like_replan(ref_traj_set[i], solution, solution, 1, infeasibility);
      //     ref_traj_set[i].states[0].v = 0;
      //     ref_traj_set[i].states[0].a = 0;
      //     ref_traj_set[i].states[0].omega = 0;
      //     ref_traj_set[i].states[0].phi = 0;
      //     planner_->Plan_diff_drive(ref_traj_set[i], solution, start, goal, solution, 1, infeasibility);
      //     config_->opti_w_a = config_->opti_w_a * config_->factor_b;
      //     DrawTrajectoryRviz(solution, config_, 1, path_pub); 
      //   }
      //   solution_set.push_back(solution);
      // }
      // for (int i = 0; i < solution_set.size(); i++) {
      //   for (int j = 0; j < solution_set[i].states.size(); j++) {
      //     sol_all.states.push_back(solution_set[i].states[j]);
      //   }
      //   time_set.push_back(solution_set[i].states.size());
      // }
      // writeTrajectoryToYAML(sol_all, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/sarah_traj.yaml");
      // writetime_count(time_set, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/sarah_time.yaml");
      ros::spinOnce();
      rate.sleep();	
	}
    return 0;
}

/**
 * file fcoord_test.cpp
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief formation coordination algorithm
 * data 2023-11-22
 * 
 * @copyright Copyroght(c) 2023
*/
#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include "heterogeneous_formation_controller/heterogeneous_formation_controller.h"
#include "heterogeneous_formation_controller/visualization/plot.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>

#include "iris/iris.h"
#include <Eigen/Core>
#include <formation_generation/timer.hpp>
#include "heterogeneous_formation_controller/yaml_all.h"
#include "heterogeneous_formation_controller/math/generate_obs.h"
#include <Eigen/Core>
#include <math.h>
#include <boost/geometry.hpp>
// #include <boost/geometry/core/point_type.hpp>
// #include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>

using namespace heterogeneous_formation_controller;
struct VertexPt {
  double x, y; 
};
struct CSpriority {
  int traj1_index;
  int traj2_index;
  int cs_begin;

  // 构造函数
  CSpriority(int traj1_ind, int traj2_ind, int cs_begin) : traj1_index(traj1_ind), traj2_index(traj2_ind), cs_begin(cs_begin) {}

  // 重载小于运算符，用于 set 的比较
  bool operator<(const CSpriority& other) const {
      return cs_begin < other.cs_begin;
  }  
};
BOOST_GEOMETRY_REGISTER_POINT_2D(VertexPt, double, boost::geometry::cs::cartesian, x, y)
typedef boost::geometry::model::d2::point_xy<double> point_type;
int cal_idle_index(math::Pose hp_pose, int inf_cs_lp, int inf_cs_hp, int sup_cs_hp, int current_ts, Trajectory_temp traj_lp,
  Trajectory_temp traj_hp, std::shared_ptr<heterogeneous_formation_controller::Environment> env, std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config);
std::vector<int>  refine_replan(const std::vector<int>& inputVector);
bool CheckCanstop(Trajectory_temp traj, int stop_pt, std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config);
std::vector<int> CheckSECollision(const std::vector<int>& arr1, const std::vector<int>& arr2);
bool CheckTrajIntersect(const std::vector<Trajectory_temp>& traj_set, std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config);
int countNumInterset(const std::vector<int>& inputVector);
int CalPairCSbegin(Trajectory_temp traj_1, Trajectory_temp traj_2, std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config);
bool compareByCSCount(const std::multiset<CSpriority>& a, const std::multiset<CSpriority>& b);
FullStates traj2fullstates(Trajectory_temp traj);
Trajectory_temp fulls2traj(const FullStates & fullstates);

bool compareByCSCount(const std::multiset<CSpriority>& a, const std::multiset<CSpriority>& b) {  
  return a.size() > b.size();
}

Trajectory_temp fulls2traj(const FullStates & fullstates) {
  Trajectory_temp traj;
  traj.resize(fullstates.states.size());
  double dt = fullstates.tf / fullstates.states.size();
  for (int i = 0; i < fullstates.states.size(); i++) {
    traj[i].x = fullstates.states[i].x;
    traj[i].y = fullstates.states[i].y;
    traj[i].theta = fullstates.states[i].theta;
    traj[i].v = fullstates.states[i].v;
    traj[i].phi = fullstates.states[i].phi;
    traj[i].a = fullstates.states[i].a;
    traj[i].omega = fullstates.states[i].omega;
    traj[i].t = i * dt;
  }
  return traj;
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

void DrawTrajectoryRviz(const FullStates sol_traj, std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config,
      int robot_index, ros::Publisher path_pub) {
  for (int i = 0; i < sol_traj.states.size(); i++) {
    auto x0_disc = config->vehicle.GetVertexPositions(sol_traj.states[i].x, sol_traj.states[i].y, sol_traj.states[i].theta, 1.0);
    std::vector<double> x1, y1;
    x1.push_back(x0_disc[0]);x1.push_back(x0_disc[2]);x1.push_back(x0_disc[4]);x1.push_back(x0_disc[6]);x1.push_back(x0_disc[0]);
    y1.push_back(x0_disc[1]);y1.push_back(x0_disc[3]);y1.push_back(x0_disc[5]);y1.push_back(x0_disc[7]);y1.push_back(x0_disc[1]);
    auto color = visualization::Color::White;
    color.set_alpha(0.1);
    visualization::Plot(x1, y1, 0.2, color, i, "spatial_envelopes" + std::to_string(robot_index));
  }
  visualization::Trigger();  
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
    color.set_alpha(0.4);
    visualization::PlotPolygon(math::Polygon2d(box), 0.05, color, i, robot_name);
  }
  path_pub.publish(msg);
  visualization::Trigger();
}

std::vector<int> refine_replan(const std::vector<int>& inputVector) {
    std::vector<int> result;
    for (size_t i = 0; i < inputVector.size() - 1; ++i) {
        result.push_back(inputVector[i]);  // 将当前元素添加到结果中

        // 检查相邻元素的差值
        int diff = inputVector[i + 1] - inputVector[i];

        if (diff > 1) {
            // 插入适当的值，确保相邻元素之间的差值最大为 1
            for (int j = 1; j < diff; ++j) {
                result.push_back(inputVector[i] + j);
            }
        }
    }

    // 添加最后一个元素
    result.push_back(inputVector.back());

    return result;
}


std::vector<int> CheckSECollision(const std::vector<int>& arr1, const std::vector<int>& arr2) {
    std::vector<int> result;

    size_t i = 0, j = 0;

    while (i < arr1.size() && j < arr2.size()) {
        if (arr1[i] == arr2[j]) {
            // 当找到相等的元素时，开始记录公共区间
            int commonStart = arr1[i];

            // 移动指针，找到公共区间的结束
            while (i < arr1.size() && j < arr2.size() && arr1[i] == arr2[j]) {
                ++i;
                ++j;
            }

            // 记录公共区间的结束
            int commonEnd = arr1[i - 1];

            // 更新结果
            result.push_back(commonStart);
            result.push_back(commonEnd);
        } else if (arr1[i] < arr2[j]) {
            // 如果 arr1[i] 小于 arr2[j]，移动指针 i
            ++i;
        } else {
            // 如果 arr2[j] 小于 arr1[i]，移动指针 j
            ++j;
        }
    }

    return result;
}

bool CheckTrajIntersect(const std::vector<Trajectory_temp>& traj_set, std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config) {
  for (int i = 0; i < traj_set.size(); i++) {
    for (int j = i + 1; j < traj_set.size(); j++) {
      int traj_length = traj_set[i].size() > traj_set[j].size() ? traj_set[i].size() : traj_set[j].size();
      Trajectory_temp temp_i, temp_j;
      temp_i = traj_set[i];
      temp_j = traj_set[j];
      for (int add = traj_set[i].size(); add < traj_length; add++) {
        temp_i.push_back(traj_set[i].back());
      }
      for (int add = traj_set[j].size(); add < traj_length; add++) {
        temp_j.push_back(traj_set[j].back());
      }
      for (int k = 0; k < traj_length; k++) {
        auto x0_disc = config->vehicle.GetVertexPositions(temp_i[k].x, temp_i[k].y, temp_i[k].theta, 1.0);
        auto x1_disc = config->vehicle.GetVertexPositions(temp_j[k].x, temp_j[k].y, temp_j[k].theta, 1.0);
        std::vector<VertexPt> points1 = {{x0_disc[6], x0_disc[7]}, {x0_disc[4], x0_disc[5]}, {x0_disc[2], x0_disc[3]}, {x0_disc[0], x0_disc[1]}, {x0_disc[6], x0_disc[7]}};
        std::vector<VertexPt> points2 = {{x1_disc[6], x1_disc[7]}, {x1_disc[4], x1_disc[5]}, {x1_disc[2], x1_disc[3]}, {x1_disc[0], x1_disc[1]}, {x1_disc[6], x1_disc[7]}};
        boost::geometry::model::polygon<VertexPt> polygon1, polygon2;
        boost::geometry::assign_points(polygon1, points1); 
        boost::geometry::assign_points(polygon2, points2); 
        if (boost::geometry::intersects(polygon1, polygon2)) {
          return true;
        }
      }
    }
  }
  return false;
}

int countNumInterset(const std::vector<int>& inputVector) {
    if (inputVector.empty()) {
        return 0; // 如果输入为空，直接返回 0
    }

    int continuousSegments = 1; // 初始连续段个数为 1
    for (size_t i = 1; i < inputVector.size(); ++i) {
        // 检查相邻元素的差值是否为 1
        if (inputVector[i] - inputVector[i - 1] == 1) {
            // 如果差值为 1，继续属于同一个连续段
            continue;
        } else {
            // 如果差值不为 1，说明一个连续段结束，增加计数器
            ++continuousSegments;
        }
    }

    return continuousSegments;
}

int CalPairCSbegin(Trajectory_temp traj_1, Trajectory_temp traj_2, std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config) {
    std::vector<boost::geometry::model::polygon<VertexPt>> poly_set;
    for (int i = 0; i < traj_2.size(); i++) {
      auto x0_disc = config->vehicle.GetVertexPositions(traj_2[i].x, traj_2[i].y, traj_2[i].theta, 1.0);
      
      std::vector<VertexPt> points = {{x0_disc[6], x0_disc[7]}, {x0_disc[4], x0_disc[5]}, {x0_disc[2], x0_disc[3]}, {x0_disc[0], x0_disc[1]}, {x0_disc[6], x0_disc[7]}};
      boost::geometry::model::polygon<VertexPt> polygon;
      boost::geometry::assign_points(polygon, points); 
      poly_set.push_back(polygon);
    }
    for (int i = 0; i < traj_1.size(); i++) {
      auto x0_disc = config->vehicle.GetVertexPositions(traj_1[i].x, traj_1[i].y, traj_1[i].theta, 1.0);
      std::vector<VertexPt> points = {{x0_disc[6], x0_disc[7]}, {x0_disc[4], x0_disc[5]}, {x0_disc[2], x0_disc[3]}, {x0_disc[0], x0_disc[1]}, {x0_disc[6], x0_disc[7]}};
      boost::geometry::model::polygon<VertexPt> polygon;
      boost::geometry::assign_points(polygon, points); 
      for (int j = 0; j < poly_set.size(); j++) {
        if (boost::geometry::intersects(poly_set[j], polygon)) {
          return i;
        }
      }
    }
    return -1;
}

bool CheckCanstop(Trajectory_temp traj, int stop_pt, std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config) {
  double path_length = 0.0;
  double t_end = traj[stop_pt].t;
  double traverse_length = 0.0;
  for (int i = 1; i < stop_pt; i++) {
    path_length += hypot(traj[i].x - traj[i-1].x, traj[i].y - traj[i-1].y);
  }
  traverse_length = (t_end + t_end - 2 * config->vehicle.max_velocity / config->vehicle.max_acceleration) * config->vehicle.max_velocity / 2;
  if (traverse_length < path_length) {
    return false;
  }
  return true;
}

void RemoveRedundentCS(const int traj_1_index, const int traj_2_index, Trajectory_temp& traj_1, Trajectory_temp& traj_2, 
  const std::vector<std::vector<std::set<math::Pose>>>& critical_section_sets, std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config) {
  Trajectory_temp traj_1_, traj_2_;
  std::vector<Trajectory_temp> traj_check;
  traj_1_ = traj_1;
  traj_2_ = traj_2;
  if (critical_section_sets[traj_1_index][traj_2_index].empty() && !critical_section_sets[traj_2_index][traj_1_index].empty()) {
    for (const auto& critical_section_set : critical_section_sets[traj_2_index][traj_1_index]) {
      auto it = std::find_if(traj_2_.begin(), traj_2_.end(), [&](TrajectoryPoint_temp point) {
        return point.x == critical_section_set.x() && point.y == critical_section_set.y() && point.theta == critical_section_set.theta(); });    
      traj_2_.erase(std::remove_if(it + 1, traj_2_.end(), [&](TrajectoryPoint_temp point) { 
        return point.x == critical_section_set.x() && point.y == critical_section_set.y() && point.theta == critical_section_set.theta(); }), traj_2_.end());
      traj_check.push_back(traj_1_); traj_check.push_back(traj_2_);
      if (!CheckTrajIntersect(traj_check, config)) {
        ROS_ERROR("Redundent cs constraints found: robot%d yield to robot%d", traj_2_index, traj_1_index);
        traj_2 = traj_2_;
      }    
      else {
        traj_2_ = traj_2;
        ROS_WARN("No redundent constraints are found!");
      }
    }
  }
  else if (!critical_section_sets[traj_1_index][traj_2_index].empty() && critical_section_sets[traj_2_index][traj_1_index].empty()) {
    for (const auto& critical_section_set : critical_section_sets[traj_1_index][traj_2_index]) {
      auto it = std::find_if(traj_1_.begin(), traj_1_.end(), [&](TrajectoryPoint_temp point) {
        return point.x == critical_section_set.x() && point.y == critical_section_set.y() && point.theta == critical_section_set.theta(); });
      traj_1_.erase(std::remove_if(it + 1, traj_1_.end(), [critical_section_set](TrajectoryPoint_temp& point) { 
        return point.x == critical_section_set.x() && point.y == critical_section_set.y() && point.theta == critical_section_set.theta(); }), traj_1_.end());
      traj_check.push_back(traj_1_); traj_check.push_back(traj_2_);
      if (!CheckTrajIntersect(traj_check, config)) {
        ROS_ERROR("Redundent cs constraints found: robot%d yield to robot%d", traj_1_index, traj_2_index);
        traj_1 = traj_1_;
      }
      else {
        traj_1_ = traj_1;
        ROS_WARN("No redundent constraints are found between %d and %d!", traj_1_index, traj_2_index);
      }
    }
  }
}

std::vector<std::vector<Trajectory_temp>> UpdatePairTraj(std::shared_ptr<heterogeneous_formation_controller::hmfpcLocalPlanner> planner, 
  std::shared_ptr<heterogeneous_formation_controller::Environment> env, std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config, int traj_1_index, int traj_2_index,
    std::vector<heterogeneous_formation_controller::TrajectoryPoint_temp> traj_1, std::vector<heterogeneous_formation_controller::TrajectoryPoint_temp> traj_2, 
    std::vector<std::vector<int>>& priority_set_all,
    std::vector<std::vector<double>>& idle_sets, int loop_count, bool& no_coordination, std::vector<Trajectory_temp>& traj_set_ref,
    std::vector<std::vector<std::set<math::Pose>>>& critical_section_sets,
    const std::vector<std::vector<Trajectory_temp>>& traj_set_cur,
    std::vector<std::vector<Trajectory_temp>>& traj_set_orig) {
    std::vector<int> critical_section, critical_section12, critical_section21;
    std::vector<std::vector<Trajectory_temp>> traj_set;
    std::vector<boost::geometry::model::polygon<VertexPt>> poly_set;
    FullStates replan_ref, replan_result, solution_diff_drive, solution_car_like;
    FullStates leader_h, solution_diff_drive_h, solution_car_like_h;
    Trajectory_temp replan_diff_drive, replan_car_like, traj_diff_drive_h, traj_car_like_h;
    TrajectoryPoint start, goal;
    start.x = 0;
    start.y = 0;
    start.theta = 0;
    goal.x = 0;
    goal.y = 0;
    goal.theta = 0;
    double dt = traj_1[1].t - traj_1[0].t;
    traj_set.resize(2, std::vector<Trajectory_temp>(3));
    traj_set[0][0] = traj_1; traj_set[1][0] = traj_2;
    std::vector<Trajectory_temp> traj_set_check;
    traj_set_check.push_back(traj_1); traj_set_check.push_back(traj_2);
    if (!CheckTrajIntersect(traj_set_check, config)) {
      no_coordination = true;
      ROS_WARN("No collision, do not need to coordinate!");
      RemoveRedundentCS(traj_1_index, traj_2_index, traj_1, traj_2, critical_section_sets, config);
      // traj_set.push_back(traj_1);
      // traj_set.push_back(traj_2);
      // auto traj_fol_diff1 = generate_ref_traj_diff(traj_1, hypot(-18-(-20), -21.0083668325-(-20)), atan2(21.0083668325-20, 20-18));
      // auto traj_fol_diff1h = generate_ref_traj_diff(traj_2, hypot(-18-(-20), -21.0083668325-(-20)), atan2(21.0083668325-20, 20-18));
      // auto ref_traj1 = traj2fullstates(traj_1);
      // auto ref_traj2 = traj2fullstates(traj_2);
      // if(!planner->Plan_diff_drive(traj_fol_diff1, start, goal, solution_diff_drive, 1)) {
      //   ROS_WARN("diff_drive fail!");
      // }
      // replan_diff_drive = fulls2traj(solution_diff_drive);
      // if (!planner->Plan_car_like(ref_traj1, 2.016733665, solution_car_like)) {
      //   ROS_WARN("car-like fail!");
      // }
      // replan_car_like = fulls2traj(solution_car_like);
      // if(!planner->Plan_diff_drive(traj_fol_diff1h, start, goal, solution_diff_drive_h, 1)) {
      //   ROS_WARN("diff_drive_h fail!");
      // }
      // traj_diff_drive_h = fulls2traj(solution_diff_drive_h);
      // if (!planner->Plan_car_like(ref_traj2, 2.016733665, solution_car_like_h)) {
      //   ROS_WARN("car-like_h fail!");
      // }
      // traj_car_like_h = fulls2traj(solution_car_like_h);
      traj_set[0][0] = traj_1;
      // traj_set[0][1] = replan_diff_drive;
      // traj_set[0][2] = replan_car_like;
      traj_set[1][0] = traj_2;
      // traj_set[1][1] = traj_diff_drive_h;
      // traj_set[1][2] = traj_car_like_h;
      ROS_WARN("Collision-free between robot %d and robot %d", traj_1_index, traj_2_index);
      return traj_set;
    }
    no_coordination = false;
    ROS_WARN("Two trajectories intersect, start coordination!");
    // treat 2 as obs
    for (int i = 0; i < traj_2.size(); i++) {
      auto x0_disc = config->vehicle.GetVertexPositions(traj_2[i].x, traj_2[i].y, traj_2[i].theta, 1.0);
      
      std::vector<VertexPt> points = {{x0_disc[6], x0_disc[7]}, {x0_disc[4], x0_disc[5]}, {x0_disc[2], x0_disc[3]}, {x0_disc[0], x0_disc[1]}, {x0_disc[6], x0_disc[7]}};
      boost::geometry::model::polygon<VertexPt> polygon;
      boost::geometry::assign_points(polygon, points); 
      poly_set.push_back(polygon);
    }
    for (int i = 0; i < traj_1.size(); i++) {
      auto x0_disc = config->vehicle.GetVertexPositions(traj_1[i].x, traj_1[i].y, traj_1[i].theta, 1.0);
      std::vector<VertexPt> points = {{x0_disc[6], x0_disc[7]}, {x0_disc[4], x0_disc[5]}, {x0_disc[2], x0_disc[3]}, {x0_disc[0], x0_disc[1]}, {x0_disc[6], x0_disc[7]}};
      boost::geometry::model::polygon<VertexPt> polygon;
      boost::geometry::assign_points(polygon, points); 
      for (int j = 0; j < poly_set.size(); j++) {
        if (boost::geometry::intersects(poly_set[j], polygon)) {
          critical_section12.push_back(i);
          break;
        }
      }
    }
    poly_set.clear();
    // treat 1 as obs
    for (int i = 0; i < traj_1.size(); i++) {
      auto x0_disc = config->vehicle.GetVertexPositions(traj_1[i].x, traj_1[i].y, traj_1[i].theta, 1.0);
      std::vector<VertexPt> points = {{x0_disc[6], x0_disc[7]}, {x0_disc[4], x0_disc[5]}, {x0_disc[2], x0_disc[3]}, {x0_disc[0], x0_disc[1]}, {x0_disc[6], x0_disc[7]}};
      boost::geometry::model::polygon<VertexPt> polygon;
      boost::geometry::assign_points(polygon, points); 
      poly_set.push_back(polygon);
    }
    for (int i = 0; i < traj_2.size(); i++) {
      auto x0_disc = config->vehicle.GetVertexPositions(traj_2[i].x, traj_2[i].y, traj_2[i].theta, 1.0);
      std::vector<VertexPt> points = {{x0_disc[6], x0_disc[7]}, {x0_disc[4], x0_disc[5]}, {x0_disc[2], x0_disc[3]}, {x0_disc[0], x0_disc[1]},{x0_disc[6], x0_disc[7]}};
      boost::geometry::model::polygon<VertexPt> polygon;
      boost::geometry::assign_points(polygon, points); 
      for (int j = 0; j < poly_set.size(); j++) {
        if (boost::geometry::intersects(poly_set[j], polygon)) {
          critical_section21.push_back(i);
          break;
        }
      }
    }
    std::vector<int> intersect_se = CheckSECollision(critical_section12, critical_section21);
    if (intersect_se.empty()) {
      ROS_WARN("Two spatial envelopes do not intersect!");
      return traj_set;
    }
    // precedence calculation
    int swap_stop = 1, swap_diff = 1, swap_occupy2 = 1, swap_occupy1 = 1, swap_replanner = 1;
    bool canstop_1 = CheckCanstop(traj_1, intersect_se[0], config);
    bool canstop_2 = CheckCanstop(traj_2, intersect_se[0], config);
    // if (!canstop_1 && !canstop_2) {
    //   ROS_ERROR("No robot can stop in the entrance of cs!");
    // }
    if (canstop_2 && !canstop_1) {
      auto temp_traj = traj_1;
      traj_1 = traj_2;
      traj_2 = temp_traj;
      auto temp_critical_section = critical_section12;
      critical_section12 = critical_section21;
      critical_section21 = temp_critical_section;
      int temp_traj_index = traj_1_index;
      traj_1_index = traj_2_index;
      traj_2_index = temp_traj_index;
      swap_stop = -1;
      ROS_WARN("Priority swap: robot1 is unable to stop!");
    }
    if (critical_section12[0] == 0) {
      auto temp_traj = traj_1;
      traj_1 = traj_2;
      traj_2 = temp_traj;
      auto temp_critical_section = critical_section12;
      critical_section12 = critical_section21;
      critical_section21 = temp_critical_section;
      int temp_traj_index = traj_1_index;
      traj_1_index = traj_2_index;
      traj_2_index = temp_traj_index;
      swap_occupy1 = -1;     
      ROS_WARN("Priority swap: inf of robot1 is blocked!");
    }
    if (critical_section21[critical_section21.size() - 1] == traj_2.size() - 1) {
      auto temp_traj = traj_1;
      traj_1 = traj_2;
      traj_2 = temp_traj;
      auto temp_critical_section = critical_section12;
      critical_section12 = critical_section21;
      critical_section21 = temp_critical_section;
      int temp_traj_index = traj_1_index;
      traj_1_index = traj_2_index;
      traj_2_index = temp_traj_index;
      swap_occupy1 = -1;     
      ROS_WARN("Priority swap: sup of robot2 is blocked!");
    }
    if (critical_section12[0] < critical_section21[0] && canstop_1 == canstop_2 && 
      critical_section12[critical_section12.size() - 1] != traj_1.size() - 1 && critical_section21[0] != 0) {
      auto temp_traj = traj_1;
      traj_1 = traj_2;
      traj_2 = temp_traj;
      auto temp_critical_section = critical_section12;
      critical_section12 = critical_section21;
      critical_section21 = temp_critical_section;
      int temp_traj_index = traj_1_index;
      traj_1_index = traj_2_index;
      traj_2_index = temp_traj_index;
      swap_diff = -1;
      ROS_WARN("Priority swap: robot1 enter the critical section first!");
    }
    int num_intersect1 = countNumInterset(critical_section12);
    int num_intersect2 = countNumInterset(critical_section21);
    if (num_intersect1 < num_intersect2 && critical_section12[critical_section12.size() - 1] != traj_2.size() - 1 && critical_section21[0] != 0) {
      auto temp_traj = traj_1;
      traj_1 = traj_2;
      traj_2 = temp_traj;
      auto temp_critical_section = critical_section12;
      critical_section12 = critical_section21;
      critical_section21 = temp_critical_section;
      int temp_traj_index = traj_1_index;
      traj_1_index = traj_2_index;
      traj_2_index = temp_traj_index;
      swap_occupy2 = -1;     
      ROS_WARN("Priority swap: robot2 intersects more!");
    }
    if (critical_section12[critical_section12.size() - 1] != traj_1.size() - 1 && critical_section21[0] != 0 && idle_sets[traj_1_index][0] > idle_sets[traj_2_index][0]) {
      auto temp_traj = traj_1;
      traj_1 = traj_2;
      traj_2 = temp_traj;
      auto temp_critical_section = critical_section12;
      critical_section12 = critical_section21;
      critical_section21 = temp_critical_section;
      int temp_traj_index = traj_1_index;
      traj_1_index = traj_2_index;
      traj_2_index = temp_traj_index;
      swap_replanner = -1;     
      ROS_WARN("Priority swap: robot1 has been replanned more!");     
    }
    if (critical_section12[critical_section12.size() - 1] != traj_1.size() - 1 && critical_section21[0] != 0 && idle_sets[traj_1_index][0] == idle_sets[traj_2_index][0] && idle_sets[traj_1_index][1] > idle_sets[traj_2_index][1] && 
      idle_sets[traj_1_index][0] != 0 && idle_sets[traj_2_index][0] != 0) { 
      auto temp_traj = traj_1;
      traj_1 = traj_2;
      traj_2 = temp_traj;
      auto temp_critical_section = critical_section12;
      critical_section12 = critical_section21;
      critical_section21 = temp_critical_section;
      int temp_traj_index = traj_1_index;
      traj_1_index = traj_2_index;
      traj_2_index = temp_traj_index;
      swap_replanner = -1;     
      ROS_WARN("Priority swap: robot1 has been replanned more!");           
    }
    ROS_WARN("Traj %d with start = (%.6f, %.6f) has higher priority than traj %d with start = (%.6f, %.6f)", traj_2_index, traj_2[0].x, traj_2[0].y, traj_1_index, traj_1[0].x, traj_1[0].y);
    critical_section.push_back(std::max(critical_section12[0] - 2, 0)); critical_section.push_back(std::min(critical_section12.back() + 2, int(traj_1.size() - 2)));
    poly_set.clear();
    // find idle positoin for robot with higher priority with inf_pi_Cij
    auto x0_disc = config->vehicle.GetVertexPositions(traj_1[critical_section12[0]].x, traj_1[critical_section12[0]].y, traj_1[critical_section12[0]].theta, 1.0);
    std::vector<VertexPt> points1 = {{x0_disc[6], x0_disc[7]}, {x0_disc[4], x0_disc[5]}, {x0_disc[2], x0_disc[3]}, {x0_disc[0], x0_disc[1]}, {x0_disc[6], x0_disc[7]}};
    boost::geometry::model::polygon<VertexPt> polygon1; 
    boost::geometry::assign_points(polygon1, points1); 
    for (int i = traj_2.size() - 1; i >= 0; i--) {
      auto x0_disc = config->vehicle.GetVertexPositions(traj_2[i].x, traj_2[i].y, traj_2[i].theta, 1.0);
      std::vector<VertexPt> points = {{x0_disc[6], x0_disc[7]}, {x0_disc[4], x0_disc[5]}, {x0_disc[2], x0_disc[3]}, {x0_disc[0], x0_disc[1]}, {x0_disc[6], x0_disc[7]}};
      boost::geometry::model::polygon<VertexPt> polygon;
      boost::geometry::assign_points(polygon, points); 
      if (boost::geometry::intersects(polygon1, polygon)) {
        while (traj_2[i].x == traj_2[i-1].x && traj_2[i].y == traj_2[i-1].y && traj_2[i].theta == traj_2[i-1].theta) {
          i--;
        }
        critical_section.push_back(std::min(i + 5, int(traj_2.size() - 2)));
        break;
      }
    }
    // x0_disc = config->vehicle.GetSpatialEnvelopes(traj_1[critical_section12[critical_section12.size() - 1]].x, traj_1[critical_section12[critical_section12.size() - 1]].y, traj_1[critical_section12[critical_section12.size() - 1]].theta);
    // polys.push_back(math::Polygon2d({{x0_disc[0], x0_disc[1]}, {x0_disc[2], x0_disc[3]}, 
    // {x0_disc[4], x0_disc[5]}, {x0_disc[6], x0_disc[7]}}));
    // find idle positoin for robot with higher priority with sup_pi_Cij
    x0_disc = config->vehicle.GetVertexPositions(traj_1[critical_section12[critical_section12.size() - 1]].x, traj_1[critical_section12[critical_section12.size() - 1]].y, traj_1[critical_section12[critical_section12.size() - 1]].theta, 1.0);
    std::vector<VertexPt> points2 = {{x0_disc[6], x0_disc[7]}, {x0_disc[4], x0_disc[5]}, {x0_disc[2], x0_disc[3]}, {x0_disc[0], x0_disc[1]}, {x0_disc[6], x0_disc[7]}};
    boost::geometry::model::polygon<VertexPt> polygon2; 
    boost::geometry::assign_points(polygon2, points2); 
    for (int i = traj_2.size() - 1; i >= 0; i--) {
      auto x0_disc = config->vehicle.GetVertexPositions(traj_2[i].x, traj_2[i].y, traj_2[i].theta, 1.0);
      std::vector<VertexPt> points = {{x0_disc[6], x0_disc[7]}, {x0_disc[4], x0_disc[5]}, {x0_disc[2], x0_disc[3]}, {x0_disc[0], x0_disc[1]}, {x0_disc[6], x0_disc[7]}};
      boost::geometry::model::polygon<VertexPt> polygon;
      boost::geometry::assign_points(polygon, points); 
      if (boost::geometry::intersects(polygon2, polygon)) {
        while (traj_2[i].x == traj_2[i-1].x && traj_2[i].y == traj_2[i-1].y && traj_2[i].theta == traj_2[i-1].theta) {
          i--;
        }
        critical_section.push_back(std::min(i + 5, int(traj_2.size() - 2)));
        break;
      }
    }
    // 
    std::vector<heterogeneous_formation_controller::TrajectoryPoint_temp> replan_traj;
    std::vector<int> temp_index_set_old;
    std::vector<int> temp_index_set;
    int idle_free_orig = -1;
    int temp_index, temp_index_old = -1;
    int idle_index;
    int num_duplicate = 0, temp_duplicate_index = -1, duplicate_count = 0;
    int inf_cs_hp = critical_section[2];
    int sup_cs_hp = critical_section[3];
    std::vector<int> duplicate_set;
    duplicate_set.resize(traj_1.size() - 1, 0);
    for (int i = 0; i < traj_2.size(); i++) {
      math::Pose hp_pose(traj_2[i].x, traj_2[i].y, traj_2[i].theta);
      temp_index = cal_idle_index(hp_pose, critical_section[0], inf_cs_hp, sup_cs_hp, i, traj_1, traj_2, env, config);
      if (i >= critical_section[0]) {
        traj_1[i].t = traj_2_index;
      }
      if (temp_index == traj_1.size() - 1) {
        idle_index = temp_index_old;
        break;
      }
      if (temp_index < temp_index_old) {
        temp_index = temp_index_old;
      }
      // replan_traj.push_back(traj_1[temp_index]);
      // else if (temp_index != temp_index_old && num_duplicate >= 20 && temp_index_old == temp_duplicate_index) {
        // num_duplicate = 0;
        // temp_duplicate_index = -1;
      // }
      // else if (temp_index == temp_index_old && traj_1_index == 1) {
      //   num_duplicate++;
      //   temp_num_duplicate = 0;
      // }
      // else {
      //   if (temp_num_duplicate != 0) {
      //     for (int duplicate_ind = temp_index_old + 1; duplicate_ind < traj_1.size(); duplicate_ind++) {
      //       if (traj_1[duplicate_ind].x == traj_1[temp_index_old].x && traj_1[duplicate_ind].y == traj_1[temp_index_old].y
      //           && traj_1[duplicate_ind].theta == traj_1[temp_index_old].theta) {
      //         num_duplicate++;
      //       }
      //       else {
      //         break;
      //       }
      //     }
      //     duplicate_count = std::max(num_duplicate, temp_num_duplicate + 1);
      //     for (int duplicate_count_ = 0; duplicate_count_ < duplicate_count; duplicate_count_++) {
      //       temp_index_set.push_back(temp_index_old);
      //     }
      //     num_duplicate = 0;
      //     temp_num_duplicate = 0;
      //     temp_index_set.push_back(temp_index);
      //   }
      //   else {
      //     temp_index_set.push_back(temp_index);
      //   }
      // }
      temp_index_set.push_back(temp_index);
      temp_index_old = temp_index;
    }
    int currentElement = temp_index_set[0];
    int currentCount = 1;

    for (size_t i = 1; i < temp_index_set.size(); ++i) {
      if (temp_index_set[i] == currentElement) {
          currentCount++;
      } 
      else {
        if (currentCount > 10) {
          math::Pose temp_pose(traj_1[currentElement].x, traj_1[currentElement].y, traj_1[currentElement].theta);
          critical_section_sets[traj_1_index][traj_2_index].insert(temp_pose);
        }
        currentCount = 1;
        currentElement = temp_index_set[i];
      }
      if (currentCount > 10) {
          math::Pose temp_pose(traj_1[currentElement].x, traj_1[currentElement].y, traj_1[currentElement].theta);
          critical_section_sets[traj_1_index][traj_2_index].insert(temp_pose);
      }
    }
    // for (int i = 0; i < traj_1.size(); i++) {
    //   if (duplicate_set[i] != 0) {
    //     for (int j = 0; j < traj_1.size(); j++) {
    //       if (traj_1[j].x == traj_1[i].x && traj_1[i].y == traj_1[i].y
    //           && traj_1[j].theta == traj_1[i].theta) {
    //         num_duplicate++;
    //       }
    //       else {
    //         break;
    //       }
    //     }
    //     if (duplicate_set[i] > num_duplicate && num_duplicate != 0) {
    //       ROS_ERROR("OMG");
    //     }
    //   }
    // }

    // for (int i = idle_index + 1; i < traj_1.size(); i++) {
    //   // replan_traj.push_back(traj_1[i]);
    //   temp_index_set.push_back(i);
    // }
    for (int i = 0; i < traj_set_orig[traj_1_index][0].size(); i++) {
      if (traj_set_orig[traj_1_index][0][i].x == traj_1[idle_index].x && traj_set_orig[traj_1_index][0][i].y == traj_1[idle_index].y
        && traj_set_orig[traj_1_index][0][i].theta == traj_1[idle_index].theta) {
          idle_free_orig = i;
          break;
        }
    }

    auto replan_index_set = refine_replan(temp_index_set);
    for (int i = 0; i < replan_index_set.size(); i++) {
      replan_traj.push_back(traj_1[replan_index_set[i]]);
    }
    for (int i = idle_free_orig; i < traj_set_orig[traj_1_index][0].size(); i++) {
      traj_set_orig[traj_1_index][0][i].t = -1;
      replan_traj.push_back(traj_set_orig[traj_1_index][0][i]);
    }
    // time_index_sets[traj_1_index] = replan_index_set;
    replan_ref.states.resize(replan_traj.size());
    for (int i = 0; i < replan_traj.size(); i++) {
      replan_ref.tf = (replan_traj.size() - 1) * dt;
      replan_ref.states[i].x = replan_traj[i].x;
      replan_ref.states[i].y = replan_traj[i].y;
      replan_ref.states[i].theta = replan_traj[i].theta;
      replan_ref.states[i].v = replan_traj[i].v;
      replan_ref.states[i].phi = replan_traj[i].phi;
      replan_ref.states[i].a = replan_traj[i].a;
      replan_ref.states[i].omega = replan_traj[i].omega;
    }
    leader_h.states.resize(traj_2.size());
    for (int i = 0; i < traj_2.size(); i++) {
      leader_h.tf = (traj_2.size() - 1) * dt;
      leader_h.states[i].x = traj_2[i].x;
      leader_h.states[i].y = traj_2[i].y;
      leader_h.states[i].theta = traj_2[i].theta;
      leader_h.states[i].v = traj_2[i].v;
      leader_h.states[i].phi = traj_2[i].phi;
      leader_h.states[i].a = traj_2[i].a;
      leader_h.states[i].omega = traj_2[i].omega;
    }
    double infeasibility;
    
    // replan trajectories for car-like leaders
    
    // if(!planner->Plan_car_like_replan(replan_ref, replan_result, 1, infeasibility)) {
    //   ROS_WARN("replan fail!");
    // }
    // auto traj_leader = fulltotraj(replan_result);
    
    // auto traj_fol_diff1 = generate_ref_traj_diff(traj_leader, hypot(-18-(-20), -21.0083668325-(-20)), atan2(21.0083668325-20, 20-18));
    // auto traj_fol_diff1h = generate_ref_traj_diff(traj_2, hypot(-18-(-20), -21.0083668325-(-20)), atan2(21.0083668325-20, 20-18));
    // if(!planner->Plan_diff_drive(traj_fol_diff1, start, goal, solution_diff_drive, 1)) {
    //   ROS_WARN("diff_drive fail!");
    // }
    // replan_diff_drive = fulls2traj(solution_diff_drive);
    // if (!planner->Plan_car_like(replan_result, 2.016733665, solution_car_like)) {
    //   ROS_WARN("car-like fail!");
    // }
    // replan_car_like = fulls2traj(solution_car_like);
    // if(!planner->Plan_diff_drive(traj_fol_diff1h, start, goal, solution_diff_drive_h, 1)) {
    //   ROS_WARN("diff_drive_h fail!");
    // }
    // traj_diff_drive_h = fulls2traj(solution_diff_drive_h);
    // if (!planner->Plan_car_like(leader_h, 2.016733665, solution_car_like_h)) {
    //   ROS_WARN("car-like_h fail!");
    // }
    // traj_car_like_h = fulls2traj(solution_car_like_h);
    // leader_h.tf = replan_ref.tf; solution_diff_drive_h.tf = replan_ref.tf; solution_car_like_h.tf = replan_ref.tf; 
    // for (int i = leader_h.states.size() - 1; i < replan_ref.states.size(); i++) {
    //   leader_h.states.push_back(leader_h.states.back());
    //   solution_diff_drive_h.states.push_back(solution_diff_drive_h.states.back());
    //   solution_car_like_h.states.push_back(solution_car_like_h.states.back());
    // }
    // writeTrajectoryToYAML(replan_result, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_oppositing/replan.yaml");
    // writeTrajectoryToYAML(solution_diff_drive, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_oppositing/replan_diff.yaml");
    // writeTrajectoryToYAML(solution_car_like, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_oppositing/replan_car.yaml");
    // writeTrajectoryToYAML(leader_h, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_oppositing/leader_h.yaml");
    // writeTrajectoryToYAML(solution_diff_drive_h, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_oppositing/solution_diff_drive_h.yaml");
    // writeTrajectoryToYAML(solution_car_like_h, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_oppositing/solution_car_like_h.yaml");

    // writeTrajectoryToYAML(replan_result, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_following/replan.yaml");
    // writeTrajectoryToYAML(solution_diff_drive, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_following/replan_diff.yaml");
    // writeTrajectoryToYAML(solution_car_like, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_following/replan_car.yaml");
    // writeTrajectoryToYAML(leader_h, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_following/leader_h.yaml");
    // writeTrajectoryToYAML(solution_diff_drive_h, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_following/solution_diff_drive_h.yaml");
    // writeTrajectoryToYAML(solution_car_like_h, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_following/solution_car_like_h.yaml");

    // writeTrajectoryToYAML(replan_result, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/three_robots_linear/replan.yaml");
    // writeTrajectoryToYAML(solution_diff_drive, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/three_robots_linear/replan_diff.yaml");
    // writeTrajectoryToYAML(solution_car_like, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/three_robots_linear/replan_car.yaml");
    // writeTrajectoryToYAML(leader_h, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/three_robots_linear/leader_h.yaml");
    // writeTrajectoryToYAML(solution_diff_drive_h, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/three_robots_linear/solution_diff_drive_h.yaml");
    // writeTrajectoryToYAML(solution_car_like_h, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/three_robots_linear/solution_car_like_h.yaml");
    // writeTrajectoryToYAML(replan_result, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/three_robots_linear/replan.yaml");
    // writeTrajectoryToYAML(solution_diff_drive, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/three_robots_linear/replan_diff1.yaml");
    // writeTrajectoryToYAML(solution_car_like, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/three_robots_linear/replan_car1.yaml");
    // writeTrajectoryToYAML(leader_h, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/three_robots_linear/replan2.yaml");
    // writeTrajectoryToYAML(solution_diff_drive_h, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/three_robots_linear/replan_diff2.yaml");
    // writeTrajectoryToYAML(solution_car_like_h, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/three_robots_linear/replan_car2.yaml");
    if (swap_stop * swap_diff * swap_occupy2 * swap_occupy1 * swap_replanner == -1) {
      traj_set[0][0] = traj_2;
      // traj_set[0][1] = traj_diff_drive_h;
      // traj_set[0][2] = traj_car_like_h;
      traj_set[1][0] = replan_traj;
      // traj_set[1][0] = traj_leader;
      // traj_set[1][1] = replan_diff_drive;
      // traj_set[1][2] = replan_car_like;
      traj_set_check[0] = traj_2;
      traj_set_check[1] = replan_traj;
      // idle_sets[traj_2_index] = 1;
    }
    else {
      traj_set[0][0] = replan_traj;
      // traj_set[0][0] = traj_leader;
      // traj_set[0][1] = replan_diff_drive;
      // traj_set[0][2] = replan_car_like;
      traj_set[1][0] = traj_2;
      // traj_set[1][1] = traj_diff_drive_h;
      // traj_set[1][2] = traj_car_like_h;
      traj_set_check[0] = replan_traj;
      traj_set_check[1] = traj_2;
      // idle_sets[traj_1_index] = 1;
    }
    RemoveRedundentCS(traj_1_index, traj_2_index, traj_set[0][0], traj_set[1][0], critical_section_sets, config);
    idle_sets[traj_1_index][0]++;
    idle_sets[traj_2_index][0]++;
    idle_sets[traj_1_index][1] += 1 / loop_count;
    idle_sets[traj_2_index][1] += 1 / loop_count;
    if (CheckTrajIntersect(traj_set_check, config)) {
      ROS_ERROR("Still have collision between robot %d and robot %d", traj_1_index, traj_2_index);
    }
    else {
      ROS_WARN("Collision-free between robot %d and robot %d", traj_1_index, traj_2_index);
    }
    return traj_set;
}

void GenerateOptimalTrajectory(std::vector<std::vector<Trajectory_temp>>& traj_set,
  std::vector<double>& dt_set,  
  std::shared_ptr<heterogeneous_formation_controller::hmfpcLocalPlanner> planner) {
  
  FullStates replan_ref, replan_result, solution_diff_drive, solution_car_like;
  FullStates leader_h, solution_diff_drive_h, solution_car_like_h;
  Trajectory_temp replan_diff_drive, replan_car_like, traj_diff_drive_h, traj_car_like_h;
  TrajectoryPoint start, goal;
  start.x = 0;
  start.y = 0;
  start.theta = 0;
  goal.x = 0;
  goal.y = 0;
  goal.theta = 0;
  double max_error;
  double infeasibility = 0.0;
  std::vector<bool> success;
  success.resize(traj_set.size());
  int count = 0;
  // while (!std::all_of(success.begin(), success.end(), [](bool value) {return value;})) {
  while (count == 0) {
    // if (count != 0) {
    //   for (int i = 0; i < traj_set.size(); i++) {
    //     dt_set[i] = 1.3 * dt_set[i];
    //   }
    // }
    for (int num_leader = 0; num_leader < traj_set.size(); num_leader++) {
      double dt = dt_set[num_leader];
      replan_ref.states.resize(traj_set[num_leader][0].size());
      for (int i = 0; i < traj_set[num_leader][0].size(); i++) {
        replan_ref.tf = (traj_set[num_leader][0].size() - 1) * dt;
        replan_ref.states[i].x = traj_set[num_leader][0][i].x;
        replan_ref.states[i].y = traj_set[num_leader][0][i].y;
        replan_ref.states[i].theta = traj_set[num_leader][0][i].theta;
        replan_ref.states[i].v = traj_set[num_leader][0][i].v;
        replan_ref.states[i].phi = traj_set[num_leader][0][i].phi;
        replan_ref.states[i].a = traj_set[num_leader][0][i].a;
        replan_ref.states[i].omega = traj_set[num_leader][0][i].omega;
      }
      // leader_h.states.resize(traj_2.size());
      // for (int i = 0; i < traj_2.size(); i++) {
      //   leader_h.tf = (traj_2.size() - 1) * dt;
      //   leader_h.states[i].x = traj_2[i].x;
      //   leader_h.states[i].y = traj_2[i].y;
      //   leader_h.states[i].theta = traj_2[i].theta;
      //   leader_h.states[i].v = traj_2[i].v;
      //   leader_h.states[i].phi = traj_2[i].phi;
      //   leader_h.states[i].a = traj_2[i].a;
      //   leader_h.states[i].omega = traj_2[i].omega;
      // }
      double solution_car_like_time;
      replan_result.states.clear();
      replan_result.tf = 1e-31;
      if(!planner->Plan_car_like_replan(replan_ref, replan_result, replan_result, num_leader, infeasibility, solution_car_like_time)) {
        ROS_ERROR("replan fail!");
      }
      if (infeasibility > 0.01) {
        ROS_ERROR("Infeasibility too high, refine the trajectory!");
      }
      // auto traj_leader = traj_set[num_leader][0];
      auto traj_leader = fulls2traj(replan_result);
      // auto traj_fol_diff1 = generate_ref_traj_diff(traj_leader, 1.0*hypot(-18-(-20), -21.0083668325-(-20)), atan2(21.0083668325-20, 20-18));
      if (num_leader == 1 || num_leader == 3 || num_leader == 4) {
        double solve_time_diff;
        auto traj_fol_diff1 = generate_ref_traj_diff(traj_leader, 3.0, atan2(10.0-10.0, 10.0-8.0));
        auto traj_fol_diff2 = generate_ref_traj_diff(traj_leader, hypot(3.0, 3.0), atan2(12.016733665-10, 10-8));
        if(!planner->Plan_diff_drive(traj_fol_diff1, traj_fol_diff1, start, goal, traj_fol_diff1, 1, max_error, solve_time_diff)) {
          ROS_ERROR("diff_drive fail, increase terminal time and try again!!");
          success[num_leader] = false;
        }
        else {
          success[num_leader] = true;
        }
        if(!planner->Plan_diff_drive(traj_fol_diff2, traj_fol_diff2, start, goal, traj_fol_diff2, 1, max_error, solve_time_diff)) {
          ROS_ERROR("diff_drive fail, increase terminal time and try again!!");
          success[num_leader] = false;
        }
        else {
          success[num_leader] = true;
        }
        double solve_time_car = 0.0;
        if (!planner->Plan_car_like(replan_result, 3.0, solution_car_like, solve_time_diff)) {
          ROS_ERROR("car-like fail!");
        }
        traj_set[num_leader][0] = fulls2traj(replan_result);
        traj_set[num_leader][1] = fulls2traj(traj_fol_diff1);
        traj_set[num_leader][2] = fulls2traj(traj_fol_diff2);
        traj_set[num_leader][3] = fulls2traj(solution_car_like);
        count++;
      }
      else {
        auto traj_fol_diff1 = generate_ref_traj_diff(traj_leader, 3.0, atan2(21.0083668325-20, 20-18));
        double solve_time_diff;
        // auto traj_fol_diff1h = generate_ref_traj_diff(traj_2, hypot(-18-(-20), -21.0083668325-(-20)), atan2(21.0083668325-20, 20-18));
        if(!planner->Plan_diff_drive(traj_fol_diff1, traj_fol_diff1, start, goal, traj_fol_diff1, num_leader, max_error, solve_time_diff) || max_error > 0.5) {
          ROS_ERROR("diff_drive fail, increase terminal time and try again!!");
          success[num_leader] = false;
        }
        else {
          success[num_leader] = true;
        }
        replan_diff_drive = fulls2traj(traj_fol_diff1);
        double solve_time_car = 0.0;
        // replan_result = traj2fullstates(traj_leader);
        // if (!planner->Plan_car_like(replan_result, 1.7*2.016733665, solution_car_like, solve_time_car)) {
        if (!planner->Plan_car_like(replan_result, 3.0, solution_car_like, solve_time_car)) {
          ROS_ERROR("car-like fail!");
        }
        // replan_car_like = fulls2traj(solution_car_like);
        // if(!planner->Plan_diff_drive(traj_fol_diff1h, start, goal, solution_diff_drive_h, 1)) {
        //   ROS_WARN("diff_drive_h fail!");
        // }
        // traj_diff_drive_h = fulls2traj(solution_diff_drive_h);
        // if (!planner->Plan_car_like(leader_h, 2.016733665, solution_car_like_h)) {
        //   ROS_WARN("car-like_h fail!");
        // }
        // traj_car_like_h = fulls2traj(solution_car_like_h);
        // leader_h.tf = replan_ref.tf; 
        // solution_diff_drive_h.tf = replan_ref.tf; 
        // solution_car_like_h.tf = replan_ref.tf; 
        // for (int i = leader_h.states.size() - 1; i < replan_ref.states.size(); i++) {
        //   leader_h.states.push_back(leader_h.states.back());
        //   solution_diff_drive_h.states.push_back(solution_diff_drive_h.states.back());
        //   solution_car_like_h.states.push_back(solution_car_like_h.states.back());
        // }
        traj_set[num_leader][0] = fulls2traj(replan_result);
        traj_set[num_leader][1] = fulls2traj(traj_fol_diff1);
        traj_set[num_leader][2] = fulls2traj(solution_car_like);
        count++;
      }
    }
  }
}

int cal_idle_index(math::Pose hp_pose, int inf_cs_lp, int inf_cs_hp, int sup_cs_hp, int current_ts, Trajectory_temp traj_lp,
  Trajectory_temp traj_hp, std::shared_ptr<heterogeneous_formation_controller::Environment> env, std::shared_ptr<heterogeneous_formation_controller::PlannerConfig> config) {
  int closest_index_lp;
  if (current_ts <= inf_cs_lp || current_ts <= inf_cs_hp) {
    return std::min(current_ts, inf_cs_lp);
  }
  if (current_ts > std::min(sup_cs_hp + 2, int(traj_hp.size() - 1))) {
    return traj_lp.size() - 1;
  }
  std::vector<boost::geometry::model::polygon<VertexPt>> poly_set;
  for (int i = current_ts; i < traj_hp.size(); i++) {
    auto x0_disc = config->vehicle.GetVertexPositions(traj_hp[i].x, traj_hp[i].y, traj_hp[i].theta, 1.0);
    std::vector<VertexPt> points = {{x0_disc[6], x0_disc[7]}, {x0_disc[4], x0_disc[5]}, {x0_disc[2], x0_disc[3]}, {x0_disc[0], x0_disc[1]}, {x0_disc[6], x0_disc[7]}};
    boost::geometry::model::polygon<VertexPt> polygon; 
    boost::geometry::assign_points(polygon, points); 
    poly_set.push_back(polygon);
  }
  auto x0_disc = config->vehicle.GetVertexPositions(traj_hp[current_ts].x, traj_hp[current_ts].y, traj_hp[current_ts].theta, 1.0);
  std::vector<VertexPt> points = {{x0_disc[6], x0_disc[7]}, {x0_disc[4], x0_disc[5]}, {x0_disc[2], x0_disc[3]}, {x0_disc[0], x0_disc[1]}, {x0_disc[6], x0_disc[7]}};
  boost::geometry::model::polygon<VertexPt> polygon_cur; 
  boost::geometry::assign_points(polygon_cur, points); 
  // poly_set.push_back(polygon);
  closest_index_lp = inf_cs_lp;
  while (closest_index_lp < traj_lp.size()) {
    bool hp_free = true;
    closest_index_lp += 1;
    auto x0_disc = config->vehicle.GetVertexPositions(traj_lp[closest_index_lp].x, traj_lp[closest_index_lp].y, traj_lp[closest_index_lp].theta, 1.0);
    std::vector<VertexPt> points = {{x0_disc[6], x0_disc[7]}, {x0_disc[4], x0_disc[5]}, {x0_disc[2], x0_disc[3]}, {x0_disc[0], x0_disc[1]}, {x0_disc[6], x0_disc[7]}};
    boost::geometry::model::polygon<VertexPt> polygon; 
    boost::geometry::assign_points(polygon, points); 
    // for (int i = 0; i < poly_set.size(); i++) {
    if (boost::geometry::intersects(polygon_cur, polygon)) {
      hp_free = false;
      break;
    }
    // }
    if (!hp_free) {
      break;
    }
  }
  if (closest_index_lp == (inf_cs_lp + 1)) {
    return inf_cs_lp;
  }
  closest_index_lp  = std::max(inf_cs_lp, closest_index_lp - 2);
  poly_set.clear();
  x0_disc = config->vehicle.GetVertexPositions(traj_lp[closest_index_lp].x, traj_lp[closest_index_lp].y, traj_lp[closest_index_lp].theta, 1.0);
  std::vector<VertexPt> points_cloest = {{x0_disc[6], x0_disc[7]}, {x0_disc[4], x0_disc[5]}, {x0_disc[2], x0_disc[3]}, {x0_disc[0], x0_disc[1]}, {x0_disc[6], x0_disc[7]}};
  boost::geometry::model::polygon<VertexPt> polygon_cloest; 
  boost::geometry::assign_points(polygon_cloest, points_cloest); 
  for (int i = current_ts; i < traj_hp.size(); i++) {
    auto x0_disc = config->vehicle.GetVertexPositions(traj_hp[i].x, traj_hp[i].y, traj_hp[i].theta, 1.0);
    std::vector<VertexPt> points = {{x0_disc[6], x0_disc[7]}, {x0_disc[4], x0_disc[5]}, {x0_disc[2], x0_disc[3]}, {x0_disc[0], x0_disc[1]}, {x0_disc[6], x0_disc[7]}};
    boost::geometry::model::polygon<VertexPt> polygon; 
    boost::geometry::assign_points(polygon, points); 
    if (boost::geometry::intersects(polygon_cloest, polygon)) {
        return inf_cs_lp;
    }
  }
  return closest_index_lp;
}


int main(int argc, char **argv) {
  Timer timer_step;
  ros::init(argc, argv, "hmfpc_test_node");
  std::vector<std::vector<Trajectory_temp>> traj_set, traj_set_orig;
  std::vector<Trajectory_temp> traj_set_ref;
  std::vector<std::vector<int>> priority_set_all;
  std::vector<std::vector<std::set<math::Pose>>> critical_section_sets;
  int num_formation = 5;
  auto config_ = std::make_shared<PlannerConfig>();
  config_->vehicle.InitializeDiscs();

  auto env = std::make_shared<Environment>(config_);
  auto planner_ = std::make_shared<hmfpcLocalPlanner>(config_, env);

  ros::NodeHandle nh;
  std::vector<ros::Publisher> path_pub_set;
  ros::Publisher path_pub_car1 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_car1", 1, false);
  ros::Publisher path_pub_car2 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_car2", 1, false);
  ros::Publisher path_pub_car3 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_car3", 1, false);
  ros::Publisher path_pub_car4 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_car4", 1, false);
  ros::Publisher path_pub_car5 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_car5", 1, false);
  ros::Publisher path_pub_car6 = nh.advertise<nav_msgs::Path>("/hmfpc_test_path_car6", 1, false);
  path_pub_set.push_back(path_pub_car1); path_pub_set.push_back(path_pub_car2); path_pub_set.push_back(path_pub_car3);
  path_pub_set.push_back(path_pub_car4); path_pub_set.push_back(path_pub_car5); path_pub_set.push_back(path_pub_car6); 
  interactive_markers::InteractiveMarkerServer server_("/hmfpc_obstacle");
  math::GenerateObstacle generateobs;
  std::vector<math::Pose> obstacle;
  std::vector<math::Polygon2d> polys, polys_orig;
  std::vector<std::vector<math::Vec2d>> poly_vertices_set;
  iris::IRISProblem iris_problem(2);
  // poly_vertices_set.push_back({{-30.5, -30}, {-30.5, 30}, {-30, 30}, {-30, -30}});
  // poly_vertices_set.push_back({{-30, 30}, {-30, 30.5}, {30, 30.5}, {30, 30}});
  // poly_vertices_set.push_back({{30, 30}, {30.5, 30}, {30.5, -30}, {30, -30}});
  // poly_vertices_set.push_back({{-30, -30}, {-30, -30.5}, {30, -30.5}, {30, -30}});

  // polys.push_back(math::Polygon2d({{-30.5, -30}, {-30.5, 30}, {-30, 30}, {-30, -30}}));
  // polys.push_back(math::Polygon2d({{-30, 30}, {-30, 30.5}, {30, 30.5}, {30, 30}}));
  // polys.push_back(math::Polygon2d({{30, 30}, {30.5, 30}, {30.5, -30}, {30, -30}}));
  // polys.push_back(math::Polygon2d({{-30, -30}, {-30, -30.5}, {30, -30.5}, {30, -30}}));
  // // obstacle.push_back({0, 0, 0});
  // obstacle.push_back({0, 17, 0});
  // obstacle.push_back({0, -17, 0});
  // obstacle.push_back({0, 18.5, 0});
  // obstacle.push_back({0, -18.5, 0});
  // polys.push_back(math::Polygon2d({{-70, -2}, {30, -2}, {30, -30}, {-70, -30}}));

  // polys.push_back(math::Polygon2d({{-14, 15}, {-12, 15}, {-12, 17}, {-14, 17}}));
  // polys.push_back(math::Polygon2d({{-14, 10}, {-12, 10}, {-12, 12}, {-14, 12}}));
  // polys.push_back(math::Polygon2d({{-28, 10}, {-26, 10}, {-26, 12}, {-28, 12}}));
  // polys.push_back(math::Polygon2d({{-28, 15}, {-26, 15}, {-26, 17}, {-28, 17}}));

  // polys.push_back(math::Polygon2d({{15, 15}, {17, 15}, {17, 17}, {15, 17}}));
  // polys.push_back(math::Polygon2d({{26, 15}, {28, 15}, {28, 17}, {26, 17}}));
  // polys.push_back(math::Polygon2d({{15, 26}, {17, 26}, {17, 28}, {15, 28}}));
  // polys.push_back(math::Polygon2d({{26, 26}, {28, 26}, {28, 28}, {26, 28}}));
  polys.push_back(math::Polygon2d({{-30.5, -30}, {-30.5, 30}, {-30, 30}, {-30, -30}}));
  polys.push_back(math::Polygon2d({{-30, 30}, {-30, 30.5}, {30, 30.5}, {30, 30}}));
  polys.push_back(math::Polygon2d({{30, 30}, {30.5, 30}, {30.5, -30}, {30, -30}}));
  polys.push_back(math::Polygon2d({{-30, -30}, {-30, -30.5}, {30, -30.5}, {30, -30}}));
  polys.push_back(math::Polygon2d({{-27, 25}, {-23, 20}, {-17, 21}, {-15, 24}}));
  polys.push_back(math::Polygon2d({{-20, 14.4}, {-16, 8}, {-6, 9.6}, {-14, 16}}));
  polys.push_back(math::Polygon2d({{-17, 0}, {-18, -6}, {-6, -7.6}, {-5.3, -1.9}}));
  polys.push_back(math::Polygon2d({{5, -8.7}, {3.7, -12.6}, {14, -20}, {15, -10}}));
  polys.push_back(math::Polygon2d({{5.6, 15.6}, {3.7, 7.8}, {17, 6.8}, {14.2, 9.6}}));
  polys.push_back(math::Polygon2d({{16.3, 27.8}, {16.5, 25}, {27.3, 25}, {27.3, 27.9}}));
  polys.push_back(math::Polygon2d({{-28.4, 3.9}, {-28.3, -3.8}, {-22.6, -4}, {-20.6, 2.5}}));
  polys.push_back(math::Polygon2d({{-27.8, -23}, {-26.4, -28.5}, {-16.2, -28.3}, {-17.5, -23.2}}));
  polys.push_back(math::Polygon2d({{23.2, -16.4}, {15.6, -26}, {23.5, -28}, {28.2, -19}}));
  polys.push_back(math::Polygon2d({{7.1, -0.3}, {6.8, -2.2}, {19, -2.66}, {20.9, -0.329}}));
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
//   writeObstacleToYAML(poly_vertices_set, "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/obs.yaml");
  env->polygons() = polys;
  // for(int i = 0; i < polys.size(); i++) {
  //   auto marker = CreateMarker(i+1, polys[i], 0.2, visualization::Color::Magenta);
  //   server_.insert(marker, interactive_cb);
  // }

  // server_.applyChanges();

  visualization::Init(nh, "odom", "/hmfpc_test_vis");

  TrajectoryPoint start, goal;
  std::vector<FullStates> solution_set;
  FullStates solution1, solution2, solution3, solution4, solution5, solution6;
  FullStates solution_car_like1, solution_diff_drive1, solution_diff_drive2;
  std::vector<double> dt_set;
  ros::Rate r(10);

//   auto start_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, [&](const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose) {
//     double x = pose->pose.pose.position.x;
//     double y = pose->pose.pose.position.y;

//     double min_distance = DBL_MAX;
//     int idx = -1;
//     for(int i = 0; i < solution.states.size(); i++) {
//       double distance = hypot(solution.states[i].x - x, solution.states[i].y - y);
//       if(distance < min_distance) {
//         min_distance = distance;
//         idx = i;
//       }
//     }

//     start = solution.states[idx];
//   });

  // auto traj_1 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_following/follow_1.yaml")
  // auto traj_2 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_following/follow_2.yaml");
  // auto traj_1 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_oppositing/opposit_1.yaml");
  // auto traj_2 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_oppositing/opposit_2.yaml");
  // auto traj_1 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/three_robots_linear/trl_3.yaml");
  // auto traj_2 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/three_robots_linear/trl_2.yaml");
  // auto traj_3 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/three_robots_linear/trl_1.yaml");
  traj_set.resize(num_formation, std::vector<Trajectory_temp>(4));
  traj_set_orig.resize(num_formation, std::vector<Trajectory_temp>(3));
  traj_set_ref.resize(num_formation);
  dt_set.resize(num_formation);
  priority_set_all.resize(num_formation, std::vector<int>(5000, -1));
  critical_section_sets.resize(num_formation, std::vector<std::set<math::Pose>>(num_formation));
  // std::string file_front = "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/five_robtos_goal_oppositting/";
  // std::string file_front = "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_following/";
  // std::string file_front = "/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/gazebo_demo/";

  std::string package_path = ros::package::getPath("heterogeneous_formation_controller");
  std::string file_front = package_path + "/traj_result/complete_demo_2/";

  ROS_INFO("Using file_front = %s", file_front.c_str());
  std::string file_back = ".yaml";
  auto traj_1 = generate_traj(file_front + "trl_1.yaml");
  traj_set[0][0] = traj_1; 
  solution1 = traj2fullstates(traj_1);
  solution_set.push_back(solution1);  
  traj_set_ref[0] = traj_set[0][0]; 
  dt_set[0] = traj_set[0][0][1].t - traj_set[0][0][0].t;
  auto traj_2 = generate_traj(file_front + "trl_2.yaml");
  // auto traj_2 = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/two_robots_oppositing/trl_2.yaml");
  solution2 = traj2fullstates(traj_2);
  traj_set[1][0] = traj_2;
  solution_set.push_back(solution2);
  traj_set_ref[1] = traj_set[1][0]; 
  dt_set[1] = traj_set[1][0][1].t - traj_set[1][0][0].t;
  auto traj_3 = generate_traj(file_front + "trl_3.yaml");
  traj_set[2][0] = traj_3;
  solution3 = traj2fullstates(traj_3);
  solution_set.push_back(solution3);
  traj_set_ref[2] = traj_set[2][0]; 
  dt_set[2] = traj_set[2][0][1].t - traj_set[2][0][0].t;
  auto traj_4 = generate_traj(file_front + "trl_4.yaml");
  traj_set[3][0] = traj_4;
  solution4 = traj2fullstates(traj_4);
  solution_set.push_back(solution4);
  traj_set_ref[3] = traj_set[3][0]; 
  dt_set[3] = traj_set[3][0][1].t - traj_set[3][0][0].t;
  auto traj_5 = generate_traj(file_front + "trl_5.yaml");
  traj_set[4][0] = traj_5;
  solution5 = traj2fullstates(traj_5);
  solution_set.push_back(solution5);
  traj_set_ref[4] = traj_set[4][0];
  dt_set[4] = traj_set[4][0][1].t - traj_set[4][0][0].t;
  // auto traj_6 = generate_traj(file_front + "trl_6.yaml");
  // traj_set[5][0] = traj_6;
  // solution6 = traj2fullstates(traj_6);
  // solution_set.push_back(solution6);
  // traj_set_ref[5] = traj_set[5][0]; 
  // dt_set[5] = traj_set[5][0][1].t - traj_set[5][0][0].t;
  traj_set_orig = traj_set;
  // @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  std::string package_path_ = ros::package::getPath("heterogeneous_formation_controller");
  auto traj_1_ori = generate_traj(package_path_ + "/traj_result/ras_demo/formation_coordination/H_environment/formation1/trl_1.yaml");
  auto sol_1_ori = traj2fullstates(traj_1_ori);
  auto traj_2_ori = generate_traj(package_path_ + "/traj_result/two_robots_oppositing/trl_2.yaml");
  auto sol_2_ori = traj2fullstates(traj_2_ori);
  std::vector<FullStates> solution_ori;
  solution_ori.push_back(sol_1_ori); solution_ori.push_back(sol_2_ori);
  // compute critical sections
  std::vector<std::multiset<CSpriority>> cs_set;
  std::multiset<CSpriority> cs_set_list;
  for (int i = 0; i < traj_set.size(); i++) {
    std::multiset<CSpriority> cs_set_i;
    for (int j = i + 1; j < traj_set.size(); j++) {
      int cs_begin = CalPairCSbegin(traj_set[i][0], traj_set[j][0], config_);
      if ( cs_begin != -1) {
        CSpriority cs_ij(i, j, cs_begin);
        cs_set_i.insert(cs_ij);
        cs_set_list.insert(cs_ij);
      }
    }
    if (!cs_set_i.empty()) {
      cs_set.push_back(cs_set_i);
    }
  }                                                                                                           
  std::sort(cs_set.begin(), cs_set.end(), compareByCSCount);
  std::vector<std::vector<double>> idle_set(num_formation, std::vector<double>(2, 0.0));
  int loop_count = 1;
  bool no_coordination;
  for (int i = 0; i < traj_set.size(); i++) {
    for (int j = 0; j < traj_set[i][0].size(); j++) {
      traj_set[i][0][j].t = -1;
    }
  }
  timer_step.reset();
  // start coordination algorithm
  while (CheckTrajIntersect(traj_set_ref, config_)) {
    for (const auto& pair_traj : cs_set_list) {
      std::vector<Trajectory_temp> temp_traj_set;
      std::vector<std::vector<Trajectory_temp>> traj_set_new = UpdatePairTraj(planner_, env, config_, pair_traj.traj1_index, pair_traj.traj2_index, traj_set[pair_traj.traj1_index][0], traj_set[pair_traj.traj2_index][0], 
        priority_set_all, idle_set, loop_count, no_coordination, traj_set_ref, critical_section_sets, traj_set, traj_set_orig);
      traj_set[pair_traj.traj1_index][0] = traj_set_new[0][0];
      // traj_set[pair_traj.traj1_index][1] = traj_set_new[0][1];
      // traj_set[pair_traj.traj1_index][2] = traj_set_new[0][2];
      traj_set[pair_traj.traj2_index][0] = traj_set_new[1][0];
      // traj_set[pair_traj.traj2_index][1] = traj_set_new[1][1];
      // traj_set[pair_traj.traj2_index][2] = traj_set_new[1][2];
      traj_set_ref[pair_traj.traj1_index] = traj_set_new[0][0];
      traj_set_ref[pair_traj.traj2_index] = traj_set_new[1][0];
      if (!no_coordination) {
          loop_count++;
      }
  }
    // std::string robot_type;
    // for (int i = 1; i < 3; i++) { 
    //   robot_type = "leader";
    //   std::string traj_leader = file_front + robot_type + std::to_string(i) + file_back;
    //   writeTrajectoryToYAML(traj2fullstates(traj_set[i][0]), traj_leader);
    //   robot_type = "diff_drive";
    //   std::string traj_diff_drive = file_front + robot_type + std::to_string(i) + file_back;
    //   writeTrajectoryToYAML(traj2fullstates(traj_set[i][1]), traj_diff_drive);
    //   robot_type = "car_like";
    //   std::string traj_car_like = file_front + robot_type + std::to_string(i) + file_back;
    //   writeTrajectoryToYAML(traj2fullstates(traj_set[i][2]), traj_car_like);
    // }
    // if (traj_set[0][1].empty()) {
    //   ROS_ERROR("EMPTY");
    // }
  }
  // remove redundent cs_constraints
  for (const auto& pair_traj : cs_set_list) {
    std::vector<Trajectory_temp> temp_traj_set;
    std::vector<std::vector<Trajectory_temp>> traj_set_new = UpdatePairTraj(planner_, env, config_, pair_traj.traj1_index, pair_traj.traj2_index, traj_set[pair_traj.traj1_index][0], traj_set[pair_traj.traj2_index][0], 
      priority_set_all, idle_set, loop_count, no_coordination, traj_set_ref, critical_section_sets, traj_set, traj_set_orig);
    traj_set[pair_traj.traj1_index][0] = traj_set_new[0][0];
    // traj_set[pair_traj.traj1_index][1] = traj_set_new[0][1];
    // traj_set[pair_traj.traj1_index][2] = traj_set_new[0][2];
    traj_set[pair_traj.traj2_index][0] = traj_set_new[1][0];
    // traj_set[pair_traj.traj2_index][1] = traj_set_new[1][1];
    // traj_set[pair_traj.traj2_index][2] = traj_set_new[1][2];
    traj_set_ref[pair_traj.traj1_index] = traj_set_new[0][0];
    traj_set_ref[pair_traj.traj2_index] = traj_set_new[1][0];
    if (!no_coordination) {
        loop_count++;
    }
  }
  // for (int i = 0; i < traj_set.size(); i++) {
  //   for (int j = 0; j < traj_set[i].size(); j++) {
  //     if (traj_set[i][0][j].x == traj_set[i][0][j+1].x && traj_set[i][0][j].x == traj_set[i][0][j+1].x && traj_set[i][0][j].theta == traj_set[i][0][j+1].theta) {

  //     }
  //   }
  // }
  timer_step.stop();
  ROS_INFO_STREAM("Coordination runtime: " << timer_step.elapsedSeconds());
  for (int i = 0; i < traj_set.size(); i++) {
    for (int j = 0; j < traj_set[i][0].size(); j++) {
      priority_set_all[i][j] = int(traj_set[i][0][j].t);
    }
  }
  writePriorityToYAML(priority_set_all, file_front + + "result/" + "priority.yaml");
  
  // // Derive trajectories for the followers using optimal control
  timer_step.reset();
  GenerateOptimalTrajectory(traj_set, dt_set, planner_);
  timer_step.stop();
  ROS_INFO_STREAM("Generate optimal trajetories runtime: " << timer_step.elapsedSeconds());
  std::vector<Trajectory_temp> traj_set_check;
  for (int i = 0; i < traj_set.size(); i++) {
    traj_set_check.push_back(traj_set[i][0]);
  }
  if (CheckTrajIntersect(traj_set_check, config_)) {
    ROS_ERROR("Coordination fails!");
    // std::vector<std::vector<Trajectory_temp>> traj_set_new = UpdatePairTraj(planner_, env, config_, traj_set[0][0], traj_set[2][0]);
  }
  else {
    ROS_WARN("Coordination success!");
  }
  // write trajectories to yaml
  std::string robot_type;
  for (int i = 0; i < traj_set.size(); i++) { 
    file_front = package_path_ + "/traj_result/complete_demo_2/";
    robot_type = "leader";
    std::string traj_leader = file_front + "result/" + robot_type + std::to_string(i) + file_back;
    writeTrajectoryToYAML(traj2fullstates(traj_set[i][0]), traj_leader);
    robot_type = "diff_drive";
    std::string traj_diff_drive = file_front + "result/" + robot_type + std::to_string(i) + file_back;
    writeTrajectoryToYAML(traj2fullstates(traj_set[i][1]), traj_diff_drive);
    robot_type = "car_like";
    std::string traj_car_like = file_front + "result/" + robot_type + std::to_string(i) + file_back;
    writeTrajectoryToYAML(traj2fullstates(traj_set[i][2]), traj_car_like);
    if (i == 1 || i == 3 || i == 4) {
      robot_type = "diff_drive1";
      std::string traj_diff_drive1 = file_front + "result/" + robot_type + std::to_string(i) + file_back;
      writeTrajectoryToYAML(traj2fullstates(traj_set[i][3]), traj_diff_drive1);    
    }
  }
  writePriorityToYAML(priority_set_all, file_front + + "result/" + "priority.yaml");
  while(ros::ok()) { 
    for (int i = 0; i < 4; i++) {
      // visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Grey, i, "Boundary"+  std::to_string(i));
    }
    for (int i = 0; i < polys.size(); i++) {
      visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Red, i, "Obstacle"+  std::to_string(i));
    }
    // auto x0_disc = config_->vehicle.GetVertexPositions(traj_set[0][0][131].x, traj_set[0][0][131].y, traj_set[0][0][131].theta);
    // polys.push_back(math::Polygon2d({{x0_disc[0], x0_disc[1]}, {x0_disc[2], x0_disc[3]}, 
    // {x0_disc[4], x0_disc[5]}, {x0_disc[6], x0_disc[7]}}));
    // x0_disc = config_->vehicle.GetVertexPositions(traj_set[1][0][131].x, traj_set[1][0][131].y, traj_set[1][0][131].theta);
    // polys.push_back(math::Polygon2d({{x0_disc[0], x0_disc[1]}, {x0_disc[2], x0_disc[3]}, 
    // {x0_disc[4], x0_disc[5]}, {x0_disc[6], x0_disc[7]}}));
    // visualization::PlotPolygon(polys[6], 0.5, visualization::Color::Magenta, 5, "Obstacle"+  std::to_string(5));
    // visualization::PlotPolygon(polys[7], 0.5, visualization::Color::Magenta, 6, "Obstacle"+  std::to_string(6));
    visualization::Trigger();   
    // auto traj_leader = generate_traj("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/traj_leader.yaml");
    // auto traj_leader = fulltotraj(solution);
    // auto traj_fol_diff1 = generate_ref_traj_diff(traj_leader, hypot(-18-(-20), -21.0083668325-(-20)), atan2(21.0083668325-20, 20-18));
    // auto traj_fol_diff1 = generate_ref_traj_diff(traj_leader, hypot(-10-(-10), -8.0-(-10.0)), atan2(10.0-10.0, 10.0-8.0));
    // auto traj_fol_diff2 = generate_ref_traj_diff(traj_leader, hypot(-10-(-12.016733665), -10.0-(-8.0)), atan2(12.016733665-10, 10-8));
    // if(!planner_->Plan_diff_drive(traj_fol_diff2, start, goal, solution_diff_drive2, 1)) {
    //   break;
    // }
    // delete_yaml("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/traj_leader.yaml");
    // delete_yaml("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/traj_diff1.yaml");
    // // delete_yaml("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/traj_diff2.yaml");
    // delete_yaml("/home/weijian/Heterogeneous_formation/src/heterogeneous_formation_controller/traj_result/traj_car1.yaml");
    for (int j = 0; j < solution_set.size(); j++) {
      DrawTrajectoryRviz(traj2fullstates(traj_set[j][0]), config_, j, path_pub_set[j]);
      // DrawTrajectoryRviz(traj2fullstates(traj_set[j][1]), config_, j*3+1, path_pub_set[j*3+1]);
      // DrawTrajectoryRviz(traj2fullstates(traj_set[j][2]), config_, j*3+2, path_pub_set[j*3+2]);
    }
    ros::spinOnce();
    r.sleep();
  }

  ros::spin();
  return 0;
}

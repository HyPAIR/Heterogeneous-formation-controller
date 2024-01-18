/**
 * file formation_planner.cpp
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief heterogeneous formation planning
 * data 2023-11-22
 * 
 * @copyright Copyroght(c) 2023
*/
#include <ros/ros.h>

#include "formation_planner/time.h"
#include "formation_planner/formation_planner.h"
#include "formation_planner/lightweight_nlp_problem.h"
#include "formation_planner/math/math_utils.h"
#include "formation_planner/visualization/plot.h"
#include "formation_planner/yaml_all.h"

namespace formation_planner {

FormationPlanner::FormationPlanner(std::shared_ptr<PlannerConfig> config, std::shared_ptr<Environment> env)
  : config_(config), env_(env), coarse_path_planner_(config, env) {
  problem_ = std::make_shared<LightweightProblem>(config_, env_);
}

FullStates FormationPlanner::GenerateGuessFromPath(const std::vector<math::Pose> &path, const TrajectoryPoint &start) {
  ROS_ASSERT_MSG(!path.empty(), "global path empty");

  size_t closest_index = 0;
  double closest_dist = path.front().DistanceTo(start.position());
  for(size_t i = 1; i < path.size(); i++) {
    double dist = path[i].DistanceTo(start.position());
    if(dist < closest_dist) {
      closest_dist = dist;
      closest_index = i;
    }
  }
  std::vector<math::Pose> pruned_path;
  std::copy(std::next(path.begin(), closest_index), path.end(), std::back_inserter(pruned_path));
  return ResamplePath(pruned_path);
}

FullStates FormationPlanner::StitchPreviousSolution(const FullStates &solution, const TrajectoryPoint &start) {
  FullStates result;
  if(solution.states.empty()) {
    return result;
  }

  size_t closest_index = 0;
  double closest_dist = hypot(solution.states.front().x - start.x, solution.states.front().y - start.y);
  for(size_t i = 1; i < solution.states.size(); i++) {
    double dist = hypot(solution.states[i].x - start.x, solution.states[i].y - start.y);
    if(dist < closest_dist) {
      closest_dist = dist;
      closest_index = i;
    }
  }

  double dt = solution.tf / solution.states.size();
  result.tf = solution.tf - closest_index * dt;
  std::copy(std::next(solution.states.begin(), closest_index), solution.states.end(), std::back_inserter(result.states));
  return result;
}

bool FormationPlanner::CheckGuessFeasibility(const FullStates &guess) {
  if(guess.states.empty()) {
    return false;
  }

  for(auto &state: guess.states) {
    if (env_->CheckPoseCollision(0.0, state.pose())) {
      return false;
    }
  }
  return true;
}

bool FormationPlanner::Plan(const FullStates &prev_sol, const TrajectoryPoint &start, const TrajectoryPoint &goal, 
      iris::IRISProblem &iris_problem, FullStates &result, double& coarse_time, double& solve_time, bool show_cr) {
  FullStates guess = StitchPreviousSolution(prev_sol, start);
  if(!CheckGuessFeasibility(guess)) {
    std::vector<math::Pose> initial_path;
    double st = GetCurrentTimestamp();
    if(!coarse_path_planner_.Plan(start.pose(), goal.pose(), initial_path)) {
      ROS_ERROR("re-plan coarse path failed!");
      return false;
    }
    guess = GenerateGuessFromPath(initial_path, start);
    coarse_time = GetCurrentTimestamp() - st;
    ROS_INFO("coarse path generation time: %f", coarse_time);
    std::vector<double> xs, ys;
    // int ind = 0;
    // for(auto &pose: initial_path) {
    //   // xs.push_back(pose.x()); ys.push_back(pose.y());
    //   auto box = config_->vehicle.GenerateBox(pose);
    //   auto color = visualization::Color::Red;
    //   // color.set_alpha(0.4);
    //   visualization::PlotPolygon(math::Polygon2d(box), 0.2, color, ind++, "coarse"+std::to_string(ind));
    // }
    // // visualization::Plot(xs, ys, 0.1, visualization::Color::Yellow, 1, "Coarse Path");
    // visualization::Trigger();
  }

  Constraints constraints;
  constraints.start = start;
  constraints.goal = goal;

  // int disc_nvar = config_->vehicle.n_disc * 2;
  int vertices_nvar = config_-> vehicle.vertices * 2;
  constraints.corridor_lb.setConstant(guess.states.size(), vertices_nvar, -inf);
  constraints.corridor_ub.setConstant(guess.states.size(), vertices_nvar, inf);
  iris::IRISOptions options;
  for(size_t i = 0; i < guess.states.size(); i++) {
    // auto disc_pos = config_->vehicle.GetDiscPositions(guess.states[i].x, guess.states[i].y, guess.states[i].theta);

    // for(int j = 0; j < config_->vehicle.n_disc; j++) {
    //   math::AABox2d box;
    //   if (!env_->GenerateCorridorBox(0.0, disc_pos[j*2], disc_pos[j*2+1], config_->vehicle.disc_radius, box)) {
    //     ROS_ERROR("%d th corridor box indexed at %zu generation failed!", j, i);
    //     return false;
    //   }

    //   // iris_problem.setSeedPoint(Eigen::Vector2d(disc_pos[j*2], disc_pos[j*2+1]));
    //   // iris::IRISRegion region = inflate_region(iris_problem, options);
    //   // auto points = region.polyhedron.generatorPoints();
    //   // std::vector<math::Vec2d> points_;
    //   // for (const auto& pts : points) {
    //   //   math::Vec2d pts_temp(pts[0], pts[1]);
    //   //   points_.push_back(pts_temp);
    //   // }

    //   auto color = visualization::Color::Green;
    //   color.set_alpha(0.1);
    //   visualization::PlotPolygon(math::Polygon2d(math::Box2d(box)), 0.05, color, i, "Corridor " + std::to_string(j));
    //   // visualization::PlotPolygon(math::Polygon2d({points_}), 0.05, color, i, "Convex Region" + std::to_string(j));

    //   constraints.corridor_lb(i, j*2) = box.min_x();
    //   constraints.corridor_lb(i, j*2+1) = box.min_y();
    //   constraints.corridor_ub(i, j*2) = box.max_x();
    //   constraints.corridor_ub(i, j*2+1) = box.max_y();
    // }
    auto f_centre = config_->vehicle.GetFormationCentre(guess.states[i].x, guess.states[i].y, guess.states[i].theta);

    math::AABox2d box;
    if (!env_->GenerateCorridorBox(0.0, f_centre[0], f_centre[1], 0.3, box)) {
      ROS_ERROR("corridor box indexed at %zu generation failed!", i);
      return false;
    }

    if (show_cr) {
      iris_problem.setSeedPoint(Eigen::Vector2d(f_centre[0], f_centre[1]));
      iris::IRISRegion region = inflate_region(iris_problem, options);
      auto points = region.polyhedron.generatorPoints();
      std::vector<math::Vec2d> points_;
      for (const auto& pts : points) {
        math::Vec2d pts_temp(pts[0], pts[1]);
        points_.push_back(pts_temp);
      }

      auto color = visualization::Color::Blue;
      color.set_alpha(0.1);
      // visualization::PlotPolygon(math::Polygon2d(math::Box2d(box)), 0.05, color, i, "Corridor " + std::to_string(i));
      visualization::PlotPolygon(math::Polygon2d({points_}), 0.05, color, i, "Convex Region" + std::to_string(i));
    }
    for(int j = 0; j < config_->vehicle.vertices; j++) {
      constraints.corridor_lb(i, j*2) = box.min_x();
      constraints.corridor_lb(i, j*2+1) = box.min_y();
      constraints.corridor_ub(i, j*2) = box.max_x();
      constraints.corridor_ub(i, j*2+1) = box.max_y();
    }
  }
 
  visualization::Trigger();

  double infeasibility;
  if(!problem_->Solve(config_->opti_w_penalty0, constraints, guess, result, infeasibility, solve_time)) {
    ROS_ERROR("solver failed!");
    return false;
  }
  std::vector<double> offset, re_phi, offset_car;
  // offset_car.push_back(2.0);
  // offset.push_back(hypot(-18-(-20), -21.0083668325-(-20))); 
  // re_phi.push_back(atan2(21.0083668325-20, 20-18)); 
  offset.push_back(2.0);
  offset.push_back(hypot(-10-(-12.016733665), -10.0-(-8.0)));
  re_phi.push_back(0.0);
  re_phi.push_back(atan2(12.016733665-10, 10-8));
  int count_exp = 0;
  while (!CheckDiffDriveKinematic(result, offset, re_phi) && !CheckCarKinematic(result, offset_car)) {
    config_->opti_t = config_->opti_t * config_->factor_a;
    config_->opti_w_diff_drive = config_->opti_w_diff_drive * config_->factor_b;
    problem_->Solve(config_->opti_w_penalty0, constraints, guess, result, infeasibility, solve_time);
    count_exp++;
    if (count_exp > 10) {
      return false;
    }
  }
  if(infeasibility > config_->opti_varepsilon_tol) {
    ROS_WARN("infeasibility = %.6f > %.6f, trajectory may not be feasible", infeasibility, config_->opti_varepsilon_tol);
  }

  return true;
}

bool FormationPlanner::Plan_car_like(const FullStates &traj_lead, const double offset, FullStates &traj_follower, double& solve_time_car) {
  double solver_st = GetCurrentTimestamp();
  double dt = traj_lead.tf / traj_lead.states.size();
	traj_follower.states.resize(traj_lead.states.size());
	for (size_t l_index = 0; l_index < traj_lead.states.size(); l_index++) {
		traj_follower.states[l_index].x = traj_lead.states[l_index].x + offset * sin(traj_lead.states[l_index].theta);
		traj_follower.states[l_index].y = traj_lead.states[l_index].y - offset * cos(traj_lead.states[l_index].theta);
		traj_follower.states[l_index].theta = traj_lead.states[l_index].theta;
		traj_follower.states[l_index].v = (1 + offset * tan(traj_lead.states[l_index].phi) / config_->vehicle.wheel_base) * traj_lead.states[l_index].v;
		traj_follower.states[l_index].phi = atan(config_->vehicle.wheel_base * tan(traj_lead.states[l_index].phi) / (config_->vehicle.wheel_base + offset * traj_lead.states[l_index].phi));	
	}
	for (size_t l_index = 1; l_index < traj_lead.states.size() - 1; l_index++) {
    traj_follower.states[l_index].a = (traj_lead.states[l_index].v - traj_lead.states[l_index - 1].v) / dt;
		traj_follower.states[l_index].omega = (traj_lead.states[l_index].phi - traj_lead.states[l_index - 1].phi) / dt;
  }
  traj_follower.states[0].a = traj_lead.states[0].a;
  traj_follower.states[0].omega = traj_lead.states[0].omega;
  traj_follower.states[traj_lead.states.size() - 1].a = traj_lead.states.back().a;
  traj_follower.states[traj_lead.states.size() - 1].omega = traj_lead.states.back().omega;
  traj_follower.tf = traj_lead.tf;
  ROS_WARN("Car-like follower feasible trajectory found!");
  solve_time_car = GetCurrentTimestamp() - solver_st;
	return true;
}

bool FormationPlanner::Plan_diff_drive(const FullStates &guess, const FullStates &prev_sol, const TrajectoryPoint &start, const TrajectoryPoint &goal, 
  FullStates &result, const int robot_index, double& infeasibility, double& solve_time) {
  Constraints constraints;
  constraints.start = start;
  constraints.goal = goal;
  double max_error;
  // double infeasibility;
  double avg_error = 0.0, error_time = 0.0;
  if(!problem_->Solve_diff_drive(config_->opti_w_penalty0, constraints, guess, prev_sol, result, infeasibility, solve_time)) {
    ROS_ERROR("solver failed!");
    return false;
  }
  ROS_WARN("Fesible trajectory is found for %dth diff-drive robot!", robot_index);
  for (int i = 0; i < result.states.size(); i++) {
    error_time = hypot(prev_sol.states[i].x - result.states[i].x, prev_sol.states[i].y - result.states[i].y);
    if (error_time > max_error) {
      max_error = error_time;
    }
    avg_error += error_time;
  }
  avg_error /= result.states.size();
  ROS_WARN("max_error: %.6f, average error: %.6f", max_error, avg_error);
  if(infeasibility > config_->opti_varepsilon_tol) {
    ROS_WARN("infeasibility = %.6f > %.6f, trajectory may not be feasible", infeasibility, config_->opti_varepsilon_tol);
  }
  return true;
}

bool FormationPlanner::Plan_car_like_replan(const FullStates &guess, const FullStates &prev_sol, FullStates &result, const int robot_index, double& infeasible, double& solution_car_like) {
  Constraints constraints;
  double infeasibility;
  double max_error = 0.0, avg_error = 0.0, error_time = 0.0;
  if(!problem_->Solve_replan(config_->opti_w_penalty0, constraints, guess, prev_sol, result, infeasibility, solution_car_like)) {
    ROS_ERROR("solver failed!");
    return false;
  }
  ROS_WARN("Fesible trajectory is found for %dth replanned robot!", robot_index);
  for (int i = 0; i < result.states.size(); i++) {
    error_time = hypot(prev_sol.states[i].x - result.states[i].x, prev_sol.states[i].y - result.states[i].y);
    if (error_time > max_error) {
      max_error = error_time;
    }
    avg_error += error_time;
  }
  avg_error /= result.states.size();
  ROS_WARN("max_error: %.6f, average error: %.6f", max_error, avg_error);
  if(infeasibility > config_->opti_varepsilon_tol) {
    ROS_WARN("robot%d: infeasibility = %.6f > %.6f, trajectory may not be feasible", robot_index, infeasibility, config_->opti_varepsilon_tol);
  }
  if(infeasibility > 0.01) {
    ROS_ERROR("trajectory need to be refined!");
  }
  infeasible = infeasibility;
  return true;
}

bool FormationPlanner::CheckCarKinematic(const FullStates &current_result, const std::vector<double> offset_car) {
  for (int idx = 0; idx < offset_car.size(); idx++) { 
    double offset_car_ = offset_car[idx];
    double current_ref_vel, current_radius;
      // check velocity
      for (int i = 0; i < current_result.states.size(); i++) {
        if (current_result.states[i].phi >= 0) {
          current_radius = config_->vehicle.wheel_base / tan(-current_result.states[i].phi);
          current_ref_vel = current_result.states[i].v * (1 + offset_car_ * (current_result.states[i].phi) / config_->vehicle.wheel_base);
          if (current_ref_vel > config_->vehicle.max_velocity) {
            ROS_WARN("No kinematic-feasible trajetory found for car-like robot!");
            return false;
          }
      }
    }
    ROS_WARN("Kinematic-feasible for car-like robot!");
  }
  // check phi: has sovled to set the -phi_max as -0.218508783
  return true;
}

bool FormationPlanner::CheckDiffDriveKinematic(const FullStates &current_result, const std::vector<double> offset, const std::vector<double> re_phi) {
  for (int idx = 0; idx < offset.size(); idx++) { 
    double offset_ = offset[idx];
    double re_phi_ = re_phi[idx];
    for (int i = 1; i < current_result.states.size(); i++) {
      double dis_ts = hypot((current_result.states[i].x + offset_ * cos(current_result.states[i].theta - re_phi_)) - (current_result.states[i - 1].x + offset_ * cos(current_result.states[i - 1].theta - re_phi_)),
      (current_result.states[i].y + offset_ * sin(current_result.states[i].theta - re_phi_)) - (current_result.states[i - 1].y + offset_ * sin(current_result.states[i - 1].theta - re_phi_)));
      double vel_ts = dis_ts / (current_result.tf / current_result.states.size());
      if ( vel_ts  > config_->vehicle.max_velocity) {
        ROS_WARN("No kinematic-feasible trajetory found for diff-drive robot!");
        return false;
      }
    }
  }
  ROS_WARN("Kinematic-feasible for diff-drive robot!");
  return true;
}

FullStates FormationPlanner::ResamplePath(const std::vector<math::Pose> &path) const {
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

      auto profile = GenerateOptimalTimeProfileSegment(station_segment, start_time);
      std::copy(profile.begin(), profile.end(), std::next(time_profile.begin(), last_idx));
      start_time = profile.back();
      last_idx = i;
    }
  }

  int nfe = std::max(config_->min_nfe, int(time_profile.back() / config_->time_step));
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

std::vector<double> FormationPlanner::GenerateOptimalTimeProfileSegment(const std::vector<double> &stations, double start_time) const {
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

}
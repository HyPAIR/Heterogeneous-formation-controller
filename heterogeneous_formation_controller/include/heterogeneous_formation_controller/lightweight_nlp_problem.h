// Created by weijian on 17/11/23.
#pragma once
#include <Eigen/Dense>
#include <memory>
#include <utility>
#include <coin/IpIpoptApplication.hpp>

#include "optimizer_interface.h"
#include "planner_config.h"
#include "environment.h"

namespace heterogeneous_formation_controller {

class LightweightProblem: public IOptimizer {
public:
  LightweightProblem(std::shared_ptr<PlannerConfig> config, std::shared_ptr<Environment> env);

  bool Solve(
      double w_inf,
      const Constraints &profile,
      const FullStates &guess,
      FullStates &result,
      double &infeasibility,
      double &solve_time) override;

  bool Solve_diff_drive(
      double w_inf,
      const Constraints &profile,
      const FullStates &guess,
      const FullStates &pre_sol,
      FullStates &result,
      double &infeasibility, 
      double &solve_time) override;

  bool Solve_replan(
      double w_inf,
      const Constraints &profile,
      const FullStates &guess,
      const FullStates &pre_sol,
      FullStates &result,
      double &infeasibility,
      double &solution_car_like) override;

  bool SolveFm(
      double w_inf, 
      const std::vector<Constraints> &profile, 
      const std::vector<FullStates> &guess, 
      std::vector<FullStates> &result, 
      double &infeasibility, 
      double& solve_time,
      std::vector<double>& height_cons) override;

private:
  Ipopt::IpoptApplication app_;

  inline FullStates ConvertVectorToStates(
      const double *x0, int nfe, const TrajectoryPoint &start, const TrajectoryPoint &goal) const {
    FullStates result;
    result.tf = x0[0];
    result.states.resize(nfe);
    result.states.front() = start;
    result.states.back() = goal;

    int ncols = NVar + config_->vehicle.n_disc * 2;
    Eigen::Map<const Eigen::ArrayXXd> states(x0 + 1, nfe-2, ncols); // skip variable tf
    for(int i = 1; i < nfe - 1; i++) {
      result.states[i] = TrajectoryPoint(states.row(i-1).leftCols<NVar>());
    }
    return result;
  }

  inline std::vector<FullStates> ConvertVectorToJointStates(
      const int num_robot, const double *x0, int nfe, const std::vector<Constraints>& profile) const {
    std::vector<FullStates> result(num_robot);
    for (int j = 0; j < num_robot; j++) {
      result[j].tf = x0[0];
      result[j].states.resize(nfe);
      result[j].states.front() = profile[j].start;
      // result[j].states.back() = profile[j].goal;

      int ncols = 3 * NVar + 8 + 32;
      Eigen::Map<const Eigen::ArrayXXd> states(x0 + 1, nfe-1, ncols); // skip variable tf
      for(int i = 1; i < nfe; i++) {
        result[j].states[i] = TrajectoryPoint(states.row(i-1).middleCols(NVar * j, 7));
      }
    }
    return result;
  }
};


}
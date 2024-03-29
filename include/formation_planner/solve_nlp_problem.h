// Created by weijian on 17/11/23.
#pragma once
#include <Eigen/Dense>
#include <memory>
#include <utility>
#include <coin/IpIpoptApplication.hpp>

#include "optimizer_interface.h"
#include "planner_config.h"
#include "environment.h"

namespace formation_planner {

class SolveNLPProblem: public IOptimizer {
public:
  SolveNLPProblem(std::shared_ptr<PlannerConfig> config, std::shared_ptr<Environment> env);

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
};


}
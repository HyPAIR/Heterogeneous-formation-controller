/**
 * file lightweight_nlp_problem.cpp
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief formulize optimal control problem
 * data 2023-11-22
 * 
 * @copyright Copyroght(c) 2023
*/
#include <adolc/adolc.h>
#include <adolc/adolc_sparse.h>
#include <coin/IpTNLP.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>

#include "heterogeneous_formation_controller/lightweight_nlp_problem.h"
#include "heterogeneous_formation_controller/time.h"
#include "heterogeneous_formation_controller/yaml_all.h"

#define tag_f 1
#define tag_g 2
#define tag_L 3

namespace heterogeneous_formation_controller {
class hmfpcIPOPTInterfaceFm : public Ipopt::TNLP
{
public:
  hmfpcIPOPTInterfaceFm(double w_inf, const std::vector<Constraints> &profile, const std::vector<FullStates> &guess, const PlannerConfig &config, const std::vector<double>& height_cons)
    : w_inf_(w_inf), profile_(profile), guess_(guess), config_(config), height_cons_(height_cons) {
    nfe_ = guess_[0].states.size();
    nrows_ = nfe_ - 1;
    int vertices = 2;
    add_var_ = num_robot * (vertices * 2) + num_robot + num_robot * (num_robot - 2);
    ncols_ = NVar * num_robot + add_var_;

    // tf + [x y theta v phi a omega x_disc_0 y_disc_0 ... x_disc_n y_disc_n] (nfe * (7 + n * 2)) + theta_end
    nvar_ = 1 + nrows_ * ncols_ + 1;
    result_.resize(nvar_);
    x0_.resize(nvar_);
  }

  virtual ~hmfpcIPOPTInterfaceFm() = default;

  hmfpcIPOPTInterfaceFm(const hmfpcIPOPTInterfaceFm&) = delete;
  hmfpcIPOPTInterfaceFm& operator=(const hmfpcIPOPTInterfaceFm&) = delete;

  virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                            Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style) {
    n = nvar_;
    m = 0;
    generate_tapes(n, m, nnz_jac_g, nnz_h_lag);
    // use the C style indexing (0-based)
    index_style = C_STYLE;
    return true;
  }

  virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                               Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u) {
    Eigen::Map<Eigen::ArrayXXd> lb_mat(x_l + 1, nrows_, ncols_);
    x_l[0] = 0.1; // tf > 0.1
    x_u[0] = inf;
    x_l[nvar_ - 1] = 1e-4;
    x_u[nvar_ - 1] = 12;

    TrajectoryPointVector lb_vec;
    Eigen::Matrix<double, NVar * num_robot, 1> lb_vec_set; 
    lb_vec << -inf, -inf, -inf,
        config_.vehicle.min_velocity,
        -config_.vehicle.phi_min,
        -config_.vehicle.max_acceleration,
        -config_.vehicle.omega_max;
    for(int i = 0; i < num_robot; i++) {
      lb_vec_set.block<NVar, 1>(7 * i, 0) = lb_vec;
    }
    lb_mat.leftCols<NVar * num_robot>() = lb_vec_set.transpose().replicate(nrows_, 1);
    for (int i = 0; i < nrows_; i++) {
      Eigen::Matrix<double, num_robot + num_robot * (num_robot - 2), 1> lb_vec_;
      if (height_cons_[i + 1] == -1) {
        for (int j = 0; j < num_robot; ++j) {
          lb_vec_(j) = config_.vehicle.min_inter_dis * config_.vehicle.min_inter_dis;
        }
        for (int j = num_robot; j < num_robot + num_robot * (num_robot - 2); ++j) {
            lb_vec_(j) = 0.0;
        }
      }
      else {
        for (int j = 0; j < num_robot; ++j) {
          lb_vec_(j) = height_cons_[i + 1] * height_cons_[i + 1];
        }
        for (int j = num_robot; j < num_robot + num_robot * (num_robot - 2); ++j) {
            lb_vec_(j) = 0.0;
        }       
      }
      // lb_mat.middleCols(NVar * 3 + i - 1, 6) = lb_vec_.transpose().replicate(1, 1);
      lb_mat.block(i, NVar * num_robot, 1, num_robot + num_robot * (num_robot - 2)) << lb_vec_.transpose();
    }
    for (int i = 0; i < num_robot; i++) {
      lb_mat.middleCols(NVar * num_robot + num_robot + num_robot * (num_robot - 2) + 4 * i, 4) = profile_[i].corridor_lb.middleRows(1, nrows_);
    }

    Eigen::Map<Eigen::ArrayXXd> ub_mat(x_u + 1, nrows_, ncols_);

    TrajectoryPointVector ub_vec;
    Eigen::Matrix<double, num_robot + num_robot * (num_robot - 2), 1> ub_vec_;
    Eigen::Matrix<double, NVar * num_robot, 1> ub_vec_set; 
    ub_vec << inf, inf, inf,
        config_.vehicle.max_velocity,
        config_.vehicle.phi_max,
        config_.vehicle.max_acceleration,
        config_.vehicle.omega_max;
    for (int j = 0; j < num_robot; ++j) {
      ub_vec_(j) = vvcm.xv2 * vvcm.xv2;
    }
    for (int j = num_robot; j < num_robot + num_robot * (num_robot - 2); ++j) {
        ub_vec_(j) = inf;
    }
    for(int i = 0; i < num_robot; i++) {
      ub_vec_set.block<NVar, 1>(7 * i, 0) = ub_vec;
    }
    ub_mat.leftCols<NVar * num_robot>() = ub_vec_set.transpose().replicate(nrows_, 1);
    ub_mat.middleCols(NVar * num_robot, num_robot + num_robot * (num_robot - 2)) = ub_vec_.transpose().replicate(nrows_, 1);
    for (int i = 0; i < num_robot; i++) {
      ub_mat.middleCols(NVar * num_robot + num_robot + num_robot * (num_robot - 2) + 4 * i, 4) = profile_[i].corridor_ub.middleRows(1, nrows_);
    }
    return true;
  }

  virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                  bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                  Ipopt::Index m, bool init_lambda,
                                  Ipopt::Number* lambda) {
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);

    Eigen::Map<Eigen::ArrayXXd> x0_mat(x + 1, nrows_, ncols_);
    auto x0_state_mat = x0_mat.leftCols<NVar * num_robot>();
    auto x0_disc_mat = x0_mat.rightCols(add_var_);
    x[0] = guess_[0].tf;
    x[nvar_ - 1] = 0.0;
    for (int i = 0; i < guess_.size(); i++) {
      x[nvar_ - 1] += pow(profile_[i].goal.x - guess_[i].states.back().x, 2)
                    + pow(profile_[i].goal.y - guess_[i].states.back().y, 2)
                    + pow(profile_[i].goal.theta - guess_[i].states.back().theta, 2);
                    // + pow(profile_[i].goal.v - guess_[i].states.back().v, 2)
                    // + pow(profile_[i].goal.phi - guess_[i].states.back().phi, 2)
                    // + pow(profile_[i].goal.a - guess_[i].states.back().a, 2)
                    // + pow(profile_[i].goal.omega - guess_[i].states.back().omega, 2);
    }

    for(int i = 0; i < nrows_; i++) {
      Eigen::VectorXd state_vecs(num_robot * 7);
      std::vector<double> pos_vec;
      std::vector<double> add_vec, all_add_vec;
      for (int j = 0; j < num_robot; j++) {
        state_vecs.segment(j * 7, 7) = guess_[j].states[i+1].vec();
        pos_vec.push_back(pow(guess_[(j+1)%num_robot].states[i+1].x - guess_[j].states[i+1].x, 2) + pow(guess_[(j+1)%num_robot].states[i+1].y - guess_[j].states[i+1].y, 2));
        // pos_vec.push_back(0.0);
        // pos_vec.push_back(guess_[j].states[i+1].x);
        // pos_vec.push_back(guess_[j].states[i+1].y);
      }
      for (int k = 0; k < num_robot; k++) {
        for (int p = 0; p < num_robot; p++) {
          if (p != k && (p + 1) % num_robot != k) {
            int Np = (p + 1) % num_robot;
            pos_vec.push_back((guess_[k].states[i+1].x - guess_[p].states[i+1].x) * (guess_[p].states[i+1].y - guess_[Np].states[i+1].y)
                        + (guess_[k].states[i+1].y - guess_[p].states[i+1].y) * (guess_[Np].states[i+1].x - guess_[p].states[i+1].x));
          }
        }
      }
      // pos_vec.push_back((guess_[0].states[i+1].x - guess_[2].states[i+1].x) * (guess_[2].states[i+1].y - guess_[1].states[i+1].y)
      //                   + (guess_[0].states[i+1].y - guess_[2].states[i+1].y) * (guess_[1].states[i+1].x - guess_[2].states[i+1].x));
      // pos_vec.push_back((guess_[1].states[i+1].x - guess_[0].states[i+1].x) * (guess_[0].states[i+1].y- guess_[2].states[i+1].y)
      //                   + (guess_[1].states[i+1].y - guess_[0].states[i+1].y) * (guess_[2].states[i+1].x - guess_[0].states[i+1].x));
      // pos_vec.push_back((guess_[2].states[i+1].x - guess_[1].states[i+1].x) * (guess_[1].states[i+1].y - guess_[0].states[i+1].y)
      //                   + (guess_[2].states[i+1].y - guess_[1].states[i+1].y) * (guess_[0].states[i+1].x - guess_[1].states[i+1].x));
      // vvcm.cak_direct_kinmatics(pos_vec, add_vec);
      // all_add_vec.insert(all_add_vec.end(), add_vec.begin(), add_vec.end());
      all_add_vec.insert(all_add_vec.end(), pos_vec.begin(), pos_vec.end());
      for (int j = 0; j < num_robot; j++) {
        auto vec_temp = config_.vehicle.GetDiscPositions(guess_[j].states[i+1].x, guess_[j].states[i+1].y, guess_[j].states[i+1].theta);
        // auto vec_temp = config_.vehicle.GetVertexPositions(guess_[j].states[i+1].x, guess_[j].states[i+1].y, guess_[j].states[i+1].theta, 1.0);
        all_add_vec.insert(all_add_vec.end(), vec_temp.begin(), vec_temp.end());
      }
      x0_state_mat.row(i) = state_vecs;
      // auto x0_disc = config_.vehicle.GetDiscPositions(guess_.states[i+1].x, guess_.states[i+1].y, guess_.states[i+1].theta);
      x0_disc_mat.row(i) = Eigen::Map<Eigen::VectorXd>(all_add_vec.data(), all_add_vec.size());
    }
    return true;
  }

  template<class T> T eval_infeasibility(const T *x) {
    T dt = x[0] / nfe_;

    T infeasibility = 0.0;
    T infeasibility_terminal = 0.0;
    // initial constraints
    for (int j = 0; j < num_robot; j++) {
      infeasibility +=  pow(x[1 + (0 + 7 * j) * nrows_ + 0] - profile_[j].start.x - dt * profile_[j].start.v * cos(profile_[j].start.theta), 2)
                      + pow(x[1 + (1 + 7 * j) * nrows_ + 0] - profile_[j].start.y - dt * profile_[j].start.v * sin(profile_[j].start.theta), 2)
                      + pow(x[1 + (2 + 7 * j) * nrows_ + 0] - profile_[j].start.theta - dt * profile_[j].start.v * tan(profile_[j].start.phi) / config_.vehicle.wheel_base, 2)
                      + pow(x[1 + (3 + 7 * j) * nrows_ + 0] - profile_[j].start.v - dt * profile_[j].start.a, 2)
                      + pow(x[1 + (4 + 7 * j) * nrows_ + 0] - profile_[j].start.phi - dt * profile_[j].start.omega, 2);
      // terminal constraints
      // infeasibility += pow(x[nvar_ - 1] - 
      //                 (pow(profile_[j].goal.x - x[1 + (0 + 7 * j) * nrows_ + nrows_-1] - dt * x[1 + (3 + 7 * j) * nrows_ + nrows_-1] * cos(x[1 + (2 + 7 * j) * nrows_ + nrows_-1]), 2)
      //                 + pow(profile_[j].goal.y - x[1 + (1 + 7 * j) * nrows_ + nrows_-1] - dt * x[1 + (3 + 7 * j) * nrows_ + nrows_-1] * sin(x[1 + (2 + 7 * j) * nrows_ + nrows_-1]), 2)
      //                 + pow(profile_[j].goal.theta - x[1 + (2 + 7 * j) * nrows_ + nrows_-1] - dt * x[1 + (3 + 7 * j) * nrows_ + nrows_-1] * tan(x[1 + (4) * nrows_ + nrows_-1]) / config_.vehicle.wheel_base, 2)
      //                 + pow(profile_[j].goal.v - x[1 + (3 + 7 * j) * nrows_ + nrows_-1] - dt * x[1 + (5 + 7 * j) * nrows_ + nrows_-1], 2)
      //                 + pow(profile_[j].goal.phi - x[1 + (4 + 7 * j) * nrows_ + nrows_-1] - dt * x[1 + (6 + 7 * j) * nrows_ + nrows_-1], 2)), 2);
      infeasibility_terminal += pow(profile_[j].goal.x - x[1 + (0 + 7 * j) * nrows_ + nrows_ - 1], 2)
                              + pow(profile_[j].goal.y - x[1 + (1 + 7 * j) * nrows_ + nrows_ - 1], 2)
                              + pow(profile_[j].goal.theta - x[1 + (2 + 7 * j) * nrows_ + nrows_ - 1], 2);
      // infeasibility += pow(sin(profile_[j].goal.theta) - sin(x[nvar_ - 1]), 2) + pow(cos(profile_[j].goal.theta) - cos(x[nvar_ - 1]), 2);
      infeasibility +=  pow(profile_[j].goal.v - x[1 + (3 + 7 * j) * nrows_ + nrows_-1], 2)
                      + pow(profile_[j].goal.phi - x[1 + (4 + 7 * j) * nrows_ + nrows_-1], 2)
                      + pow(profile_[j].goal.a - x[1 + (5 + 7 * j) * nrows_ + nrows_-1], 2)
                      + pow(profile_[j].goal.omega - x[1 + (6 + 7 * j) * nrows_ + nrows_-1], 2);
      // kinematic constraints
      for(int i = 1; i < nrows_; i++) {
        infeasibility +=  pow(x[1 + (0 + 7 * j) * nrows_ + i] - x[1 + (0 + 7 * j) * nrows_ + i-1] - dt * x[1 + (3 + 7 * j ) * nrows_ + i-1] * cos(x[1 + (2 + 7 * j) * nrows_ + i-1]), 2)
                        + pow(x[1 + (1 + 7 * j) * nrows_ + i] - x[1 + (1 + 7 * j) * nrows_ + i-1] - dt * x[1 + (3 + 7 * j ) * nrows_ + i-1] * sin(x[1 + (2 + 7 * j) * nrows_ + i-1]), 2)
                        + pow(x[1 + (2 + 7 * j) * nrows_ + i] - x[1 + (2 + 7 * j) * nrows_ + i-1] - dt * x[1 + (3 + 7 * j ) * nrows_ + i-1] * tan(x[1 + (4 + 7 * j) * nrows_ + i-1]) / config_.vehicle.wheel_base, 2)
                        + pow(x[1 + (3 + 7 * j) * nrows_ + i] - x[1 + (3 + 7 * j) * nrows_ + i-1] - dt * x[1 + (5 + 7 * j ) * nrows_ + i-1], 2)
                        + pow(x[1 + (4 + 7 * j) * nrows_ + i] - x[1 + (4 + 7 * j) * nrows_ + i-1] - dt * x[1 + (6 + 7 * j ) * nrows_ + i-1], 2);
      } 
      // collision avoidance constraints
      // for (int i = 0; i < nrows_; i++) {
      //   infeasibility +=  pow(x[1 + (21 + 6 + 4 * j + 0 * 2) * nrows_ + i] - (x[1 + (0 + 7 * j) * nrows_ + i] - config_.vehicle.front_hang_length * cos(x[1 + (2 + 7 * j) * nrows_ + i]) - config_.vehicle.width / 2 * sin(x[1 + (2 + 7 * j) * nrows_ + i])), 2)
      //                   + pow(x[1 + (21 + 6 + 4 * j + 0 * 2 + 1) * nrows_ + i] - (x[1 + (1 + 7 * j) * nrows_ + i] - config_.vehicle.front_hang_length * sin(x[1 + (2 + 7 * j) * nrows_ + i]) + config_.vehicle.width / 2 * cos(x[1 + (2 + 7 * j) * nrows_ + i])), 2)
      //                   + pow(x[1 + (21 + 6 + 4 * j + 1 * 2) * nrows_ + i] - (x[1 + (0 + 7 * j) * nrows_ + i] - config_.vehicle.front_hang_length * cos(x[1 + (2 + 7 * j) * nrows_ + i]) + config_.vehicle.width / 2 * sin(x[1 + (2 + 7 * j) * nrows_ + i])), 2)
      //                   + pow(x[1 + (21 + 6 + 4 * j + 1 * 2 + 1) * nrows_ + i] - (x[1 + (1 + 7 * j) * nrows_ + i] - config_.vehicle.front_hang_length * sin(x[1 + (2 + 7 * j) * nrows_ + i]) - config_.vehicle.width / 2 * cos(x[1 + (2 + 7 * j) * nrows_ + i])), 2)
      //                   + pow(x[1 + (21 + 6 + 4 * j + 2 * 2) * nrows_ + i] - (x[1 + (0 + 7 * j) * nrows_ + i] + config_.vehicle.rear_hang_length * cos(x[1 + (2 + 7 * j) * nrows_ + i]) - config_.vehicle.width / 2 * sin(x[1 + (2 + 7 * j) * nrows_ + i])), 2)
      //                   + pow(x[1 + (21 + 6 + 4 * j + 2 * 2 + 1) * nrows_ + i] - (x[1 + (1 + 7 * j) * nrows_ + i] + config_.vehicle.rear_hang_length * sin(x[1 + (2 + 7 * j) * nrows_ + i]) + config_.vehicle.width / 2 * cos(x[1 + (2 + 7 * j) * nrows_ + i])), 2)    
      //                   + pow(x[1 + (21 + 6 + 4 * j + 3 * 2) * nrows_ + i] - (x[1 + (0 + 7 * j) * nrows_ + i] + config_.vehicle.rear_hang_length * cos(x[1 + (2 + 7 * j) * nrows_ + i]) + config_.vehicle.width / 2 * sin(x[1 + (2 + 7 * j) * nrows_ + i])), 2)         
      //                   + pow(x[1 + (21 + 6 + 4 * j + 3 * 2 + 1) * nrows_ + i] - (x[1 + (1 + 7 * j) * nrows_ + i] + config_.vehicle.rear_hang_length * sin(x[1 + (2 + 7 * j) * nrows_ + i]) - config_.vehicle.width / 2 * cos(x[1 + (2 + 7 * j) * nrows_ + i])), 2); 
      // }
      for(int i = 0; i < nrows_; i++) {
        for (int k = 0; k < config_.vehicle.n_disc; k++) {
          infeasibility += pow(x[1 + (NVar * num_robot + num_robot + num_robot * (num_robot - 2) + 4 * j + k * 2) * nrows_ + i] - x[1 + (0 + 7 * j) * nrows_ + i] - config_.vehicle.disc_coefficients[k] * cos(x[1 + (2 + 7 * j) * nrows_ + i]), 2)
                         + pow(x[1 + (NVar * num_robot + num_robot + num_robot * (num_robot - 2) + 4 * j + k * 2 + 1) * nrows_ + i] - x[1 + (1 + 7 * j) * nrows_ + i] - config_.vehicle.disc_coefficients[k] * sin(x[1 + (2 + 7 * j) * nrows_ + i]), 2);
        }
      }
    }
    // infeasibility += infeasibility_terminal;
    infeasibility += pow(x[nvar_ - 1] - infeasibility_terminal, 2);

    // for each robot
      // relative position constraints
    for (int i = 0; i < nrows_; i++) {
      for (int j = 0; j < num_robot; j++) {
        infeasibility += pow(x[1 + (NVar * num_robot + j) * nrows_ + i] - 
                      ((x[1 + (0 + 7 * ((j + 1) % num_robot)) * nrows_ + i] - x[1 + (0 + 7 * j) * nrows_ + i]) * (x[1 + (0 + 7 * ((j + 1) % num_robot)) * nrows_ + i] - x[1 + (0 + 7 * j) * nrows_ + i]) + 
                       (x[1 + (1 + 7 * ((j + 1) % num_robot)) * nrows_ + i] - x[1 + (1 + 7 * j) * nrows_ + i]) * (x[1 + (1 + 7 * ((j + 1) % num_robot)) * nrows_ + i] - x[1 + (1 + 7 * j) * nrows_ + i])), 2);
      }
    }
    // topological constraints
    for (int i = 0; i < nrows_; i++) {
      int topo_ind = 0;
      for (int k = 0; k < num_robot; k++) {
        for (int p = 0; p < num_robot; p++) {
          if (p != k && (p + 1) % num_robot != k) {
            int Np = (p + 1) % num_robot;
            infeasibility += pow(x[1 + (NVar * num_robot + num_robot + topo_ind) * nrows_ + i] -
                            ((x[1 + (0 + 7 * k) * nrows_ + i] - x[1 + (0 + 7 * p) * nrows_ + i]) * (x[1 + (1 + 7 * p) * nrows_ + i] - x[1 + (1 + 7 * Np) * nrows_ + i]) +
                             (x[1 + (1 + 7 * k) * nrows_ + i] - x[1 + (1 + 7 * p) * nrows_ + i]) * (x[1 + (0 + 7 * Np) * nrows_ + i] - x[1 + (0 + 7 * p) * nrows_ + i])), 2);
            topo_ind++;
          }
        }
      }
    }
    // 
    // // taut constraints
    // for (int i = 0; i < nrows_; i++) {
    //   infeasibility += pow(pow(x[1 + (21 + 3) * nrows_ + i], 2) + pow(x[1 + (21 + 4) * nrows_ + i], 2) - 
    //                   (pow(x[1 + (21 + 5) * nrows_ + i], 2) + pow(x[1 + (21 + 6) * nrows_ + i], 2) + pow(x[1 + (21 + 7) * nrows_ + i] - vvcm.zr, 2)), 2)
    //                   + pow(pow(x[1 + (21 + 3) * nrows_ + i] - vvcm.xv2t, 2) + pow(x[1 + (21 + 4) * nrows_ + i], 2) - 
    //                   (pow(x[1 + (21 + 5) * nrows_ + i] - x[1 + (21 + 0) * nrows_ + i], 2) + pow(x[1 + (21 + 6) * nrows_ + i], 2) + pow(x[1 + (21 + 7) * nrows_ + i] - vvcm.zr, 2)), 2)
    //                   + pow(pow(x[1 + (21 + 3) * nrows_ + i] - vvcm.xv3t, 2) + pow(x[1 + (21 + 4) * nrows_ + i] - vvcm.yv3t, 2) - 
    //                   (pow(x[1 + (21 + 5) * nrows_ + i] - x[1 + (21 + 1) * nrows_ + i], 2) + pow(x[1 + (21 + 6) * nrows_ + i] - x[1 + (21 + 2) * nrows_ + i], 2) + pow(x[1 + (21 + 10) * nrows_ + i] - vvcm.zr, 2)), 2);
    // }
    // // plane constraints
    // for (int i = 0; i < nrows_; i++) {
    //   infeasibility += pow(x[1 + (21 + 5) * nrows_ + i] - vvcm.xv2t * x[1 + (21 + 3) * nrows_ + i] / x[1 + (21 + 0) * nrows_ + i] - (x[1 + (21 + 0) * nrows_ + i] * x[1 + (21 + 0) * nrows_ + i] - vvcm.xv2t * vvcm.xv2t) / (2 * x[1 + (21 + 0) * nrows_ + i]) , 2)
    //                  + pow(x[1 + (21 + 6) * nrows_ + i] - (vvcm.xv3t / x[1 + (21 + 2) * nrows_ + i] - (x[1 + (21 + 1) * nrows_ + i] * vvcm.xv2t) / (x[1 + (21 + 2) * nrows_ + i] * x[1 + (21 + 0) * nrows_ + i])) * x[1 + (21 + 3) * nrows_ + i]
    //                  - vvcm.yv3t * x[1 + (21 + 4) * nrows_ + i] / x[1 + (21 + 2) * nrows_ + i] 
    //                  + (x[1 + (21 + 1) * nrows_ + i] / x[1 + (21 + 2) * nrows_ + i]) * ((x[1 + (21 + 0) * nrows_ + i] * x[1 + (21 + 0) * nrows_ + i] - vvcm.xv2t * vvcm.xv2t) / (2 * x[1 + (21 + 0) * nrows_ + i]))
    //                  - (x[1 + (21 + 1) * nrows_ + i] * x[1 + (21 + 1) * nrows_ + i] + x[1 + (21 + 2) * nrows_ + i] * x[1 + (21 + 2) * nrows_ + i] - vvcm.xv3t * vvcm.xv3t - vvcm.yv3t * vvcm.yv3t) / (2 * x[1 + (21 + 2) * nrows_ + i]), 2);
    // }
    // // 3d space linear constraints
    // for (int i = 0; i < nrows_; i++) {
    //   infeasibility += pow(((vvcm.xv2t * vvcm.xv2t - x[1 + (21 + 0) * nrows_ + i] * x[1 + (21 + 0) * nrows_ + i]) / vvcm.xv2t) * x[1 + (21 + 3) * nrows_ + i] 
    //                  + (x[1 + (21 + 0) * nrows_ + i] * (vvcm.xv3t * x[1 + (21 + 0) * nrows_ + i] - x[1 + (21 + 1) * nrows_ + i] * vvcm.xv2t) / (vvcm.xv2t * vvcm.yv3t)) * x[1 + (21 + 4) * nrows_ + i] 
    //                  - (vvcm.xv2t * vvcm.xv2t - x[1 + (21 + 0) * nrows_ + i] * x[1 + (21 + 0) * nrows_ + i]) / 2, 2)
    //                  + pow((vvcm.xv3t * x[1 + (21 + 0) * nrows_ + i]) * x[1 + (21 + 3) * nrows_ + i] 
    //                  + (x[1 + (21 + 0) * nrows_ + i] * (vvcm.yv3t * vvcm.yv3t - x[1 + (21 + 2) * nrows_ + i] * x[1 + (21 + 2) * nrows_ + i]) / vvcm.yv3t) * x[1 + (21 + 4) * nrows_ + i]
    //                  - (x[1 + (21 + 1) * nrows_ + i] * (x[1 + (21 + 0) * nrows_ + i] * x[1 + (21 + 0) * nrows_ + i] - vvcm.xv2t * vvcm.xv2t) / 2 
    //                     + x[1 + (21 + 0) * nrows_ + i] * (vvcm.xv3t * vvcm.xv3t + vvcm.yv3t * vvcm.yv3t - x[1 + (21 + 1) * nrows_ + i] * x[1 + (21 + 1) * nrows_ + i] - x[1 + (21 + 2) * nrows_ + i] * x[1 + (21 + 2) * nrows_ + i]) / 2), 2);
    // }

    // for(int i = 0; i < nrows_; i++) {
    //   for (int j = 0; j < config_.vehicle.vertices; j++) {
    //     infeasibility += pow(x[1 + (NVar + j * 2) * nrows_ + i] - x[1 + (0) * nrows_ + i] - config_.vehicle.disc_coefficients[j] * cos(x[1 + (2) * nrows_ + i]), 2)
    //                      + pow(x[1 + (NVar + j * 2 + 1) * nrows_ + i] - x[1 + (1) * nrows_ + i] - config_.vehicle.disc_coefficients[j] * sin(x[1 + (2) * nrows_ + i]), 2);
    //   }
    // }
    // for (int i = 0; i < nrows_; i++) {
    //   infeasibility +=   pow(x[1 + (NVar + 0 * 2) * nrows_ + i] - x[1 + (0) * nrows_ + i] 
    //                    + hypot(config_.vehicle.rear_hang_length, config_.vehicle.width / 2) * cos(x[1 + (2) * nrows_ + i] - atan2(config_.vehicle.width / 2, config_.vehicle.rear_hang_length)), 2)
    //                    + pow(x[1 + (NVar + 0 * 2 + 1) * nrows_ + i] - x[1 + (1) * nrows_ + i] 
    //                    + hypot(config_.vehicle.rear_hang_length, config_.vehicle.width / 2) * sin(x[1 + (2) * nrows_ + i] - atan2(config_.vehicle.width / 2, config_.vehicle.rear_hang_length)), 2)
    //                    + pow(x[1 + (NVar + 1 * 2) * nrows_ + i] - x[1 + (0) * nrows_ + i]
    //                    + hypot(config_.vehicle.rear_hang_length, config_.vehicle.width / 2) * cos(x[1 + (2) * nrows_ + i] - atan2(config_.vehicle.width / 2, config_.vehicle.rear_hang_length)) - (1.2 + 2.0) * cos(x[1 + (2) * nrows_ + i]), 2)
    //                    + pow(x[1 + (NVar + 1 * 2 + 1) * nrows_ + i] - x[1 + (1) * nrows_ + i]
    //                    + hypot(config_.vehicle.rear_hang_length, config_.vehicle.width / 2) * sin(x[1 + (2) * nrows_ + i] - atan2(config_.vehicle.width / 2, config_.vehicle.rear_hang_length)) - (1.2 + config_.vehicle.offset) * sin(x[1 + (2) * nrows_ + i]), 2)
    //                    + pow(x[1 + (NVar + 2 * 2) * nrows_ + i] - x[1 + (0) * nrows_ + i] 
    //                    + hypot(config_.vehicle.rear_hang_length, config_.vehicle.width / 2) * cos(x[1 + (2) * nrows_ + i] - atan2(config_.vehicle.width / 2, config_.vehicle.rear_hang_length)) - hypot(1.2 + 2.0, 1.2 + config_.vehicle.offset) * cos(x[1 + (2) * nrows_ + i] - atan2(1.2 + config_.vehicle.offset, 1.2 + 2.0)), 2)
    //                    + pow(x[1 + (NVar + 2 * 2 + 1) * nrows_ + i] - x[1 + (1) * nrows_ + i]
    //                    + hypot(config_.vehicle.rear_hang_length, config_.vehicle.width / 2) * sin(x[1 + (2) * nrows_ + i] - atan2(config_.vehicle.width / 2, config_.vehicle.rear_hang_length)) - hypot(1.2 + 2.0, 1.2 + config_.vehicle.offset) * sin(x[1 + (2) * nrows_ + i] - atan2(1.2 + config_.vehicle.offset, 1.2 + 2.0)), 2)    
    //                    + pow(x[1 + (NVar + 3 * 2) * nrows_ + i] - x[1 + (0) * nrows_ + i]
    //                    + hypot(config_.vehicle.rear_hang_length, config_.vehicle.width / 2) * cos(x[1 + (2) * nrows_ + i] - atan2(config_.vehicle.width / 2, config_.vehicle.rear_hang_length)) - (1.2 + config_.vehicle.offset) * cos(M_PI / 2 - x[1 + (2) * nrows_ + i]), 2)         
    //                    + pow(x[1 + (NVar + 3 * 2 + 1) * nrows_ + i] - x[1 + (1) * nrows_ + i]   
    //                    + hypot(config_.vehicle.rear_hang_length, config_.vehicle.width / 2) * sin(x[1 + (2) * nrows_ + i] - atan2(config_.vehicle.width / 2, config_.vehicle.rear_hang_length)) + (1.2 + config_.vehicle.offset) * sin(M_PI / 2 - x[1 + (2) * nrows_ + i]), 2); 
    // }

    // double offset_ = hypot(-18-(-20), -21.0083668325-(-20));
    // double re_phi = atan2(21.0083668325-20, 20-18);
    // T infeasibility_temp = 0.0;
    // for(int i = 1; i < nrows_; i++) {
    //   infeasibility_temp += pow((x[1 + (0) * nrows_ + i] + offset_ * cos(x[1 + (2) * nrows_ + i] - re_phi)) - (x[1 + (0) * nrows_ + i - 1] + offset_ * cos(x[1 + (2) * nrows_ + i - 1] - re_phi)), 2) +
    //       pow((x[1 + (1) * nrows_ + i] + offset_ * sin(x[1 + (2) * nrows_ + i] - re_phi)) - (x[1 + (1) * nrows_ + i - 1] + offset_ * sin(x[1 + (2) * nrows_ + i - 1] - re_phi)), 2) / (dt * dt)
    //       - pow(config_.vehicle.max_velocity, 2);
    //         // infeasibility += pow(pow((x[1 + (0) * nrows_ + i] + offset_ * cos(x[1 + (2) * nrows_ + i] - re_phi)) - (x[1 + (0) * nrows_ + i - 1] + offset_ * cos(x[1 + (2) * nrows_ + i - 1] - re_phi)), 2) +
    //         // pow((x[1 + (1) * nrows_ + i] + offset_ * sin(x[1 + (2) * nrows_ + i] - re_phi)) - (x[1 + (1) * nrows_ + i - 1] + offset_ * sin(x[1 + (2) * nrows_ + i - 1] - re_phi)), 2) / (dt * dt)
    //         // - pow(config_.vehicle.max_velocity, 2), 2);
    //         // infeasibility += 100;
    //   }
    //   if (pow(x[1 + (2) * nrows_ + i] - x[1 + (2) * nrows_ + i - 1], 2) / (dt * dt) > pow(config_.vehicle.omega_max,2)) {
    //     infeasibility += pow(pow(x[1 + (2) * nrows_ + i] - x[1 + (2) * nrows_ + i - 1], 2) / (dt * dt) - pow(config_.vehicle.omega_max,2), 2);
    //   }
    //   T distance_diff1 = x_diff_car1 * x_diff_car1 + y_diff_car1 * y_diff_car1;
    //   T angle_diff1 = atan2(y_diff_car1, x_diff_car1);
    //   T x_diff_car2 = x[1 + (0) * nrows_ + i - 1] + offset_ * cos(x[1 + (2) * nrows_ + i - 1] - re_phi) - x[1 + (0) * nrows_ + i - 2] + offset_ * cos(x[1 + (2) * nrows_ + i - 2] - re_phi);
    //   T y_diff_car2 = x[1 + (1) * nrows_ + i - 1] + offset_ * sin(x[1 + (2) * nrows_ + i - 1] - re_phi) - x[1 + (1) * nrows_ + i - 2] + offset_ * sin(x[1 + (2) * nrows_ + i - 2] - re_phi);
    //   T distance_diff2 = x_diff_car2 * x_diff_car2 + y_diff_car2 * y_diff_car2;
    //   T angle_diff2 = atan2(y_diff_car2, x_diff_car2);
    //   T a_diff = (distance_diff1 - distance_diff2) / (dtemp * dtemp);
    //   T w_diff = (ainfeasibilityngle_diff1 - angle_diff2) / dtemp;
    //   obj_value += config_.opti_w_diff_drive * (a_diff * a_diff + w_diff * w_diff);

    return infeasibility;
  }

  template<class T> bool eval_obj(Ipopt::Index n, const T *x, T& obj_value) {
    obj_value = config_.opti_t * x[0];
    T dt = x[0] / nfe_;
    // T dtemp = x[0] / nfe_;
    // T offset_ = hypot(-18-(-20), -21.0083668325-(-20));
    // T re_phi = atan2(21.0083668325-20, 20-18);
    // for(int i = 2; i < nrows_; i++) {
    //   T x_diff_car1 = x[1 + (0) * nrows_ + i] + offset_ * cos(x[1 + (2) * nrows_ + i] - re_phi) - x[1 + (0) * nrows_ + i - 1] + offset_ * cos(x[1 + (2) * nrows_ + i - 1] - re_phi);
    //   T y_diff_car1 = x[1 + (1) * nrows_ + i] + offset_ * sin(x[1 + (2) * nrows_ + i] - re_phi) - x[1 + (1) * nrows_ + i - 1] + offset_ * sin(x[1 + (2) * nrows_ + i - 1] - re_phi);
    //   T distance_diff1 = x_diff_car1 * x_diff_car1 + y_diff_car1 * y_diff_car1;
    //   T angle_diff1 = atan2(y_diff_car1, x_diff_car1);
    //   T x_diff_car2 = x[1 + (0) * nrows_ + i - 1] + offset_ * cos(x[1 + (2) * nrows_ + i - 1] - re_phi) - x[1 + (0) * nrows_ + i - 2] + offset_ * cos(x[1 + (2) * nrows_ + i - 2] - re_phi);
    //   T y_diff_car2 = x[1 + (1) * nrows_ + i - 1] + offset_ * sin(x[1 + (2) * nrows_ + i - 1] - re_phi) - x[1 + (1) * nrows_ + i - 2] + offset_ * sin(x[1 + (2) * nrows_ + i - 2] - re_phi);
    //   T distance_diff2 = x_diff_car2 * x_diff_car2 + y_diff_car2 * y_diff_car2;
    //   T angle_diff2 = atan2(y_diff_car2, x_diff_car2);inf
    //   T a_diff = (distance_diff1 - distance_diff2) / (dtemp * dtemp);
    //   T w_diff = (angle_diff1 - angle_diff2) / dtemp;
    //   obj_value += config_.opti_w_diff_drive * (a_diff * a_diff + w_diff * w_diff);
    // }
    for (int j = 0; j < num_robot; j++) {
      for(int i = 0; i < nrows_; i++) {
        // a^2 + w^2
        obj_value += config_.opti_w_a * x[1 + (j * 7 + 5) * nrows_ + i] * x[1 + (j * 7 + 5) * nrows_ + i]
                  + config_.opti_w_omega * x[1 + (j * 7 + 6) * nrows_ + i] * x[1 + (j * 7 + 6) * nrows_ + i];
      }
    }
    // terminal constraints
    // for (int i = 0; i < guess_.size(); i++) {
    //   obj_value += pow(profile_[i].goal.x - x[1 + (0 + 7 * i) * nrows_ + nrows_-1], 2)
    //              + pow(profile_[i].goal.y - x[1 + (1 + 7 * i) * nrows_ + nrows_-1], 2)
    //              + pow(profile_[i].goal.theta - x[1 + (2 + 7 * i) * nrows_ + nrows_-1], 2)
    //              + pow(profile_[i].goal.v - x[1 + (3 + 7 * i) * nrows_ + nrows_-1], 2)
    //              + pow(profile_[i].goal.phi - x[1 + (4 + 7 * i) * nrows_ + nrows_-1], 2)
    //              + pow(profile_[i].goal.a - x[1 + (5 + 7 * i) * nrows_ + nrows_-1], 2)
    //              + pow(profile_[i].goal.omega - x[1 + (6 + 7 * i) * nrows_ + nrows_-1], 2);
    // }
    // height constraints
    // for(int i = 0; i < nrows_; i++) {
    //   obj_value += config_.opti_w_a * pow(((x[1 + (0 + 7 * 1) * nrows_ + i] - x[1 + (0 + 7 * 0) * nrows_ + i]) * (x[1 + (0 + 7 * 1) * nrows_ + i] - x[1 + (0 + 7 * 0) * nrows_ + i]) + 
    //                    (x[1 + (1 + 7 * 1) * nrows_ + i] - x[1 + (1 + 7 * 0) * nrows_ + i]) * (x[1 + (1 + 7 * 1) * nrows_ + i] - x[1 + (1 + 7 * 0) * nrows_ + i]) - vvcm.xv2t * vvcm.xv2t), 2);
    //   obj_value += config_.opti_w_a * pow(((x[1 + (0 + 7 * 2) * nrows_ + i] - x[1 + (0 + 7 * 1) * nrows_ + i]) * (x[1 + (0 + 7 * 2) * nrows_ + i] - x[1 + (0 + 7 * 1) * nrows_ + i]) + 
    //                    (x[1 + (1 + 7 * 2) * nrows_ + i] - x[1 + (1 + 7 * 1) * nrows_ + i]) * (x[1 + (1 + 7 * 2) * nrows_ + i] - x[1 + (1 + 7 * 1) * nrows_ + i]) - vvcm.xv2t * vvcm.xv2t), 2);
    //   obj_value += config_.opti_w_a * pow(((x[1 + (0 + 7 * 0) * nrows_ + i] - x[1 + (0 + 7 * 2) * nrows_ + i]) * (x[1 + (0 + 7 * 0) * nrows_ + i] - x[1 + (0 + 7 * 2) * nrows_ + i]) + 
    //                    (x[1 + (1 + 7 * 0) * nrows_ + i] - x[1 + (1 + 7 * 2) * nrows_ + i]) * (x[1 + (1 + 7 * 0) * nrows_ + i] - x[1 + (1 + 7 * 2) * nrows_ + i]) - vvcm.xv2t * vvcm.xv2t), 2);
    // }
    // for (int i = 0; i < nrows_; i++) {
    //   obj_value += config_.opti_w_a * pow(x[1 + (21 + 0) * nrows_ + i] - vvcm.xv2t * vvcm.xv2t, 2);
    //   obj_value += config_.opti_w_a * pow(x[1 + (21 + 1) * nrows_ + i] - vvcm.xv2t * vvcm.xv2t, 2);
    //   obj_value += config_.opti_w_a * pow(x[1 + (21 + 2) * nrows_ + i] - vvcm.xv2t * vvcm.xv2t, 2);
    // }
    // obj_value += x[nvar_ - 1] * x[nvar_ - 1];
    // for(int i = 0; i < nrows_; i++) {
    //   obj_value += pow(pow(x[1 + 3 * nrows_ + i] - 0.6, 2) + pow(x[1 + 4 * nrows_ + i] - 0.6 * sqrt(1 / 3), 2), 2);
    // }

    // for(int i = 1; i < nrows_; i++) {
    //   obj_value += config_.opti_w_diff_drive * x[1 + 3 * nrows_ + i] * x[1 + 3 * nrows_ + i];

    //   // obj_value += config_.opti_w_diff_drive*(pow((x[1 + (0) * nrows_ + i] + offset_ * cos(x[1 + (2) * nrows_ + i] - re_phi)) - (x[1 + (0) * nrows_ + i - 1] + offset_ * cos(x[1 + (2) * nrows_ + i - 1] - re_phi)), 2) +
    //   //     pow((x[1 + (1) * nrows_ + i] + offset_ * sin(x[1 + (2) * nrows_ + i] - re_phi)) - (x[1 + (1) * nrows_ + i - 1] + offset_ * sin(x[1 + (2) * nrows_ + i - 1] - re_phi)), 2) / (dt * dt));
            
    //         // infeasibility += pow(pow((x[1 + (0) * nrows_ + i] + offset_ * cos(x[1 + (2) * nrows_ + i] - re_phi)) - (x[1 + (0) * nrows_ + i - 1] + offset_ * cos(x[1 + (2) * nrows_ + i - 1] - re_phi)), 2) +
    //         // pow((x[1 + (1) * nrows_ + i] + offset_ * sin(x[1 + (2) * nrows_ + i] - re_phi)) - (x[1 + (1) * nrows_ + i - 1] + offset_ * sin(x[1 + (2) * nrows_ + i - 1] - re_phi)), 2) / (dt * dt)
    //         // - pow(config_.vehicle.max_velocity, 2), 2);
    //         // infeasibility += 100;
    //   }

    obj_value += w_inf_ * eval_infeasibility(x);
    return true;
  }

  template<class T> bool eval_constraints(Ipopt::Index n, const T *x, Ipopt::Index m, T *g) {
    return true;
  }

  virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value) {
    eval_obj(n, x, obj_value);
    return true;
  }

  virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f) {
    gradient(tag_f, n, x, grad_f);
    return true;
  }

  virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g) {
    eval_constraints(n, x, m, g);
    return true;
  }

  virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                          Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index *jCol,
                          Ipopt::Number* values) {
    if(values == nullptr) {
      // return the structure of the jacobian
      for(Ipopt::Index idx = 0; idx < nnz_jac; idx++) {
        iRow[idx] = rind_g[idx];
        jCol[idx] = cind_g[idx];
      }
    } else {
      // return the values of the jacobian of the constraints
      sparse_jac(tag_g, m, n, 1, x, &nnz_jac, &rind_g, &cind_g, &jacval, options_g);

      for(Ipopt::Index idx = 0; idx < nnz_jac; idx++) {
        values[idx] = jacval[idx];
      }
    }
    return true;
  }

  virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                      Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                      bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                      Ipopt::Index* jCol, Ipopt::Number* values) {
    if(values == nullptr) {
      // return the structure. This is a symmetric matrix, fill the lower left triangle only.
      for(Ipopt::Index idx = 0; idx < nnz_L; idx++) {
        iRow[idx] = rind_L[idx];
        jCol[idx] = cind_L[idx];
      }
    } else {
      // return the values. This is a symmetric matrix, fill the lower left triangle only
      obj_lam[0] = obj_factor;
      for(Ipopt::Index idx = 0; idx < m; idx++)
        obj_lam[1 + idx] = lambda[idx];

      set_param_vec(tag_L, m + 1, obj_lam);
      sparse_hess(tag_L, n, 1, const_cast<double *>(x), &nnz_L, &rind_L, &cind_L, &hessval, options_L);

      for(Ipopt::Index idx = 0; idx < nnz_L; idx++) {
        values[idx] = hessval[idx];
      }
    }

    return true;
  }

  virtual void finalize_solution(Ipopt::SolverReturn status,
                                 Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U,
                                 Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda,
                                 Ipopt::Number obj_value,
                                 const Ipopt::IpoptData* ip_data,
                                 Ipopt::IpoptCalculatedQuantities* ip_cq) {
    std::copy(x, x + n, result_.data());

    delete[] obj_lam;
    free(rind_g);
    free(cind_g);
    free(rind_L);
    free(cind_L);
    free(jacval);
    free(hessval);
  }


  /** Method to generate the required tapes */
  virtual void generate_tapes(Ipopt::Index n, Ipopt::Index m, Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag) {
    Ipopt::Number *lamp  = new double[m];
    Ipopt::Number *zl    = new double[m];
    Ipopt::Number *zu    = new double[m];

    adouble *xa   = new adouble[n];
    adouble *g    = new adouble[m];
    double *lam   = new double[m];
    double sig;
    adouble obj_value;

    double dummy;
    obj_lam   = new double[m+1];
    get_starting_point(n, true, x0_.data(), false, zl, zu, m, false, lamp);

    trace_on(tag_f);

    for(Ipopt::Index idx=0;idx<n;idx++)
      xa[idx] <<= x0_[idx];

    eval_obj(n,xa,obj_value);

    obj_value >>= dummy;

    trace_off();

    trace_on(tag_g);

    for(Ipopt::Index idx=0;idx<n;idx++)
      xa[idx] <<= x0_[idx];

    eval_constraints(n,xa,m,g);


    for(Ipopt::Index idx=0;idx<m;idx++)
      g[idx] >>= dummy;

    trace_off();

    trace_on(tag_L);

    for(Ipopt::Index idx=0;idx<n;idx++)
      xa[idx] <<= x0_[idx];
    for(Ipopt::Index idx=0;idx<m;idx++)
      lam[idx] = 1.0;
    sig = 1.0;

    eval_obj(n,xa,obj_value);

    obj_value *= mkparam(sig);
    eval_constraints(n,xa,m,g);

    for(Ipopt::Index idx=0;idx<m;idx++)
      obj_value += g[idx]*mkparam(lam[idx]);

    obj_value >>= dummy;

    trace_off();

    rind_g = nullptr;
    cind_g = nullptr;
    rind_L = nullptr;
    cind_L = nullptr;

    options_g[0] = 0;          /* sparsity pattern by index domains (default) */
    options_g[1] = 0;          /*                         safe mode (default) */
    options_g[2] = 0;
    options_g[3] = 0;          /*                column compression (default) */

    jacval = nullptr;
    hessval = nullptr;
    sparse_jac(tag_g, m, n, 0, x0_.data(), &nnz_jac, &rind_g, &cind_g, &jacval, options_g);

    nnz_jac_g = nnz_jac;

    options_L[0] = 0;
    options_L[1] = 1;

    sparse_hess(tag_L, n, 0, x0_.data(), &nnz_L, &rind_L, &cind_L, &hessval, options_L);
    nnz_h_lag = nnz_L;

    delete[] lam;
    delete[] g;
    delete[] xa;
    delete[] zu;
    delete[] zl;
    delete[] lamp;
  }

  std::vector<double> result_;

private:
  double w_inf_;
  int nfe_, vert_nvar_;
  int nvar_, nrows_, ncols_, add_var_;
  const std::vector<Constraints> &profile_;
  const std::vector<FullStates> &guess_;
  const PlannerConfig &config_;
  const std::vector<double>& height_cons_;
  VVCM vvcm;

  std::vector<double> x0_;

  double *obj_lam;

  //** variables for sparsity exploitation
  unsigned int *rind_g;        /* row indices    */
  unsigned int *cind_g;        /* column indices */
  double *jacval;              /* values         */
  unsigned int *rind_L;        /* row indices    */
  unsigned int *cind_L;        /* column indices */
  double *hessval;             /* values */
  int nnz_jac;
  int nnz_L;
  int options_g[4];
  int options_L[4];
};

class hmfpcIPOPTInterface : public Ipopt::TNLP
{
public:
  hmfpcIPOPTInterface(double w_inf, const Constraints &profile, const FullStates &guess, const PlannerConfig &config)
    : w_inf_(w_inf), profile_(profile), guess_(guess), config_(config) {
    nfe_ = guess_.states.size();
    nrows_ = nfe_ - 2;
    int vertices = 4;
    vert_nvar_ = vertices * 2;
    ncols_ = NVar + vert_nvar_;

    // tf + [x y theta v phi a omega x_disc_0 y_disc_0 ... x_disc_n y_disc_n] (nfe * (7 + n * 2)) + theta_end
    nvar_ = 1 + nrows_ * ncols_ + 1;
    result_.resize(nvar_);
    x0_.resize(nvar_);
  }

  virtual ~hmfpcIPOPTInterface() = default;

  hmfpcIPOPTInterface(const hmfpcIPOPTInterface&) = delete;
  hmfpcIPOPTInterface& operator=(const hmfpcIPOPTInterface&) = delete;

  virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                            Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style) {
    n = nvar_;
    m = 0;
    generate_tapes(n, m, nnz_jac_g, nnz_h_lag);
    // use the C style indexing (0-based)
    index_style = C_STYLE;
    return true;
  }

  virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                               Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u) {
    Eigen::Map<Eigen::ArrayXXd> lb_mat(x_l + 1, nrows_, ncols_);
    x_l[0] = 0.1; // tf > 0.1
    x_u[0] = inf;
    x_l[nvar_ - 1] = -inf;
    x_u[nvar_ - 1] = inf;

    TrajectoryPointVector lb_vec;
    lb_vec << -inf, -inf, -inf,
        config_.vehicle.min_velocity,
        -config_.vehicle.phi_min,
        -config_.vehicle.max_acceleration,
        -config_.vehicle.omega_max;
    lb_mat.leftCols<NVar>() = lb_vec.transpose().replicate(nrows_, 1);
    lb_mat.rightCols(vert_nvar_) = profile_.corridor_lb.middleRows(1, nrows_);

    Eigen::Map<Eigen::ArrayXXd> ub_mat(x_u + 1, nrows_, ncols_);

    TrajectoryPointVector ub_vec;
    ub_vec << inf, inf, inf,
        config_.vehicle.max_velocity,
        config_.vehicle.phi_max,
        config_.vehicle.max_acceleration,
        config_.vehicle.omega_max;
    ub_mat.leftCols<NVar>() = ub_vec.transpose().replicate(nrows_, 1);
    ub_mat.rightCols(vert_nvar_) = profile_.corridor_ub.middleRows(1, nrows_);
    return true;
  }

  virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                  bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                  Ipopt::Index m, bool init_lambda,
                                  Ipopt::Number* lambda) {
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);

    Eigen::Map<Eigen::ArrayXXd> x0_mat(x + 1, nrows_, ncols_);
    auto x0_state_mat = x0_mat.leftCols<NVar>();
    auto x0_disc_mat = x0_mat.rightCols(vert_nvar_);
    x[0] = guess_.tf;
    x[nvar_ - 1] = profile_.goal.theta;

    for(int i = 0; i < nrows_; i++) {
      x0_state_mat.row(i) = guess_.states[i+1].vec();

      // auto x0_disc = config_.vehicle.GetDiscPositions(guess_.states[i+1].x, guess_.states[i+1].y, guess_.states[i+1].theta);
      auto x0_disc = config_.vehicle.GetVertexPositions(guess_.states[i+1].x, guess_.states[i+1].y, guess_.states[i+1].theta, 1.0);
      x0_disc_mat.row(i) = Eigen::Map<Eigen::VectorXd>(x0_disc.data(), x0_disc.size());
    }
    return true;
  }

  template<class T> T eval_infeasibility(const T *x) {
    T dt = x[0] / nfe_;

    T infeasibility = 0.0;
    infeasibility += pow(x[1 + (0) * nrows_ + 0] - profile_.start.x - dt * profile_.start.v * cos(profile_.start.theta), 2)
                     + pow(x[1 + (1) * nrows_ + 0] - profile_.start.y - dt * profile_.start.v * sin(profile_.start.theta), 2)
                     + pow(x[1 + (2) * nrows_ + 0] - profile_.start.theta - dt * profile_.start.v * tan(profile_.start.phi) / config_.vehicle.wheel_base, 2)
                     + pow(x[1 + (3) * nrows_ + 0] - profile_.start.v - dt * profile_.start.a, 2)
                     + pow(x[1 + (4) * nrows_ + 0] - profile_.start.phi - dt * profile_.start.omega, 2);

    for(int i = 1; i < nrows_; i++) {
      infeasibility += pow(x[1 + (0) * nrows_ + i] - x[1 + (0) * nrows_ + i-1] - dt * x[1 + (3) * nrows_ + i-1] * cos(x[1 + (2) * nrows_ + i-1]), 2)
                       + pow(x[1 + (1) * nrows_ + i] - x[1 + (1) * nrows_ + i-1] - dt * x[1 + (3) * nrows_ + i-1] * sin(x[1 + (2) * nrows_ + i-1]), 2)
                       + pow(x[1 + (2) * nrows_ + i] - x[1 + (2) * nrows_ + i-1] - dt * x[1 + (3) * nrows_ + i-1] * tan(x[1 + (4) * nrows_ + i-1]) / config_.vehicle.wheel_base, 2)
                       + pow(x[1 + (3) * nrows_ + i] - x[1 + (3) * nrows_ + i-1] - dt * x[1 + (5) * nrows_ + i-1], 2)
                       + pow(x[1 + (4) * nrows_ + i] - x[1 + (4) * nrows_ + i-1] - dt * x[1 + (6) * nrows_ + i-1], 2);
    }

    infeasibility += pow(profile_.goal.x - x[1 + (0) * nrows_ + nrows_-1] - dt * x[1 + (3) * nrows_ + nrows_-1] * cos(x[1 + (2) * nrows_ + nrows_-1]), 2)
                     + pow(profile_.goal.y - x[1 + (1) * nrows_ + nrows_-1] - dt * x[1 + (3) * nrows_ + nrows_-1] * sin(x[1 + (2) * nrows_ + nrows_-1]), 2)
                     + pow(x[nvar_ - 1] - x[1 + (2) * nrows_ + nrows_-1] - dt * x[1 + (3) * nrows_ + nrows_-1] * tan(x[1 + (4) * nrows_ + nrows_-1]) / config_.vehicle.wheel_base, 2)
                     + pow(profile_.goal.v - x[1 + (3) * nrows_ + nrows_-1] - dt * x[1 + (5) * nrows_ + nrows_-1], 2)
                     + pow(profile_.goal.phi - x[1 + (4) * nrows_ + nrows_-1] - dt * x[1 + (6) * nrows_ + nrows_-1], 2);

    infeasibility += pow(sin(profile_.goal.theta) - sin(x[nvar_ - 1]), 2) + pow(cos(profile_.goal.theta) - cos(x[nvar_ - 1]), 2);

    // for(int i = 0; i < nrows_; i++) {
    //   for (int j = 0; j < config_.vehicle.vertices; j++) {
    //     infeasibility += pow(x[1 + (NVar + j * 2) * nrows_ + i] - x[1 + (0) * nrows_ + i] - config_.vehicle.disc_coefficients[j] * cos(x[1 + (2) * nrows_ + i]), 2)
    //                      + pow(x[1 + (NVar + j * 2 + 1) * nrows_ + i] - x[1 + (1) * nrows_ + i] - config_.vehicle.disc_coefficients[j] * sin(x[1 + (2) * nrows_ + i]), 2);
    //   }
    // }
    for (int i = 0; i < nrows_; i++) {
      infeasibility += pow(x[1 + (NVar + 0 * 2) * nrows_ + i] - x[1 + (0) * nrows_ + i] 
                       + hypot(config_.vehicle.rear_hang_length, config_.vehicle.width / 2) * cos(x[1 + (2) * nrows_ + i] - atan2(config_.vehicle.width / 2, config_.vehicle.rear_hang_length)), 2)
                       + pow(x[1 + (NVar + 0 * 2 + 1) * nrows_ + i] - x[1 + (1) * nrows_ + i] 
                       + hypot(config_.vehicle.rear_hang_length, config_.vehicle.width / 2) * sin(x[1 + (2) * nrows_ + i] - atan2(config_.vehicle.width / 2, config_.vehicle.rear_hang_length)), 2)
                       + pow(x[1 + (NVar + 1 * 2) * nrows_ + i] - x[1 + (0) * nrows_ + i]
                       + hypot(config_.vehicle.rear_hang_length, config_.vehicle.width / 2) * cos(x[1 + (2) * nrows_ + i] - atan2(config_.vehicle.width / 2, config_.vehicle.rear_hang_length)) - (1.2 + 2.0) * cos(x[1 + (2) * nrows_ + i]), 2)
                       + pow(x[1 + (NVar + 1 * 2 + 1) * nrows_ + i] - x[1 + (1) * nrows_ + i]
                       + hypot(config_.vehicle.rear_hang_length, config_.vehicle.width / 2) * sin(x[1 + (2) * nrows_ + i] - atan2(config_.vehicle.width / 2, config_.vehicle.rear_hang_length)) - (1.2 + config_.vehicle.offset) * sin(x[1 + (2) * nrows_ + i]), 2)
                       + pow(x[1 + (NVar + 2 * 2) * nrows_ + i] - x[1 + (0) * nrows_ + i] 
                       + hypot(config_.vehicle.rear_hang_length, config_.vehicle.width / 2) * cos(x[1 + (2) * nrows_ + i] - atan2(config_.vehicle.width / 2, config_.vehicle.rear_hang_length)) - hypot(1.2 + 2.0, 1.2 + config_.vehicle.offset) * cos(x[1 + (2) * nrows_ + i] - atan2(1.2 + config_.vehicle.offset, 1.2 + 2.0)), 2)
                       + pow(x[1 + (NVar + 2 * 2 + 1) * nrows_ + i] - x[1 + (1) * nrows_ + i]
                       + hypot(config_.vehicle.rear_hang_length, config_.vehicle.width / 2) * sin(x[1 + (2) * nrows_ + i] - atan2(config_.vehicle.width / 2, config_.vehicle.rear_hang_length)) - hypot(1.2 + 2.0, 1.2 + config_.vehicle.offset) * sin(x[1 + (2) * nrows_ + i] - atan2(1.2 + config_.vehicle.offset, 1.2 + 2.0)), 2)    
                       + pow(x[1 + (NVar + 3 * 2) * nrows_ + i] - x[1 + (0) * nrows_ + i]
                       + hypot(config_.vehicle.rear_hang_length, config_.vehicle.width / 2) * cos(x[1 + (2) * nrows_ + i] - atan2(config_.vehicle.width / 2, config_.vehicle.rear_hang_length)) - (1.2 + config_.vehicle.offset) * cos(M_PI / 2 - x[1 + (2) * nrows_ + i]), 2)         
                       + pow(x[1 + (NVar + 3 * 2 + 1) * nrows_ + i] - x[1 + (1) * nrows_ + i]   
                       + hypot(config_.vehicle.rear_hang_length, config_.vehicle.width / 2) * sin(x[1 + (2) * nrows_ + i] - atan2(config_.vehicle.width / 2, config_.vehicle.rear_hang_length)) + (1.2 + config_.vehicle.offset) * sin(M_PI / 2 - x[1 + (2) * nrows_ + i]), 2); 
    }
    // for (int i = 0; i < nrows_; i++) {
    //   infeasibility += pow(x[1 + (NVar + 0 * 2) * nrows_ + i] - (x[1 + (0) * nrows_ + i] - config_.vehicle.front_hang_length * cos(x[1 + (2) * nrows_ + i]) - config_.vehicle.width / 2 * sin(x[1 + (2) * nrows_ + i])), 2)
    //                    + pow(x[1 + (NVar + 0 * 2 + 1) * nrows_ + i] - (x[1 + (1) * nrows_ + i] - config_.vehicle.front_hang_length * sin(x[1 + (2) * nrows_ + i]) + config_.vehicle.width / 2 * cos(x[1 + (2) * nrows_ + i])), 2)
    //                    + pow(x[1 + (NVar + 1 * 2) * nrows_ + i] - (x[1 + (0) * nrows_ + i] - config_.vehicle.front_hang_length * cos(x[1 + (2) * nrows_ + i]) + config_.vehicle.width / 2 * sin(x[1 + (2) * nrows_ + i])), 2)
    //                    + pow(x[1 + (NVar + 1 * 2 + 1) * nrows_ + i] - (x[1 + (1) * nrows_ + i] - config_.vehicle.front_hang_length * sin(x[1 + (2) * nrows_ + i]) - config_.vehicle.width / 2 * cos(x[1 + (2) * nrows_ + i])), 2)
    //                    + pow(x[1 + (NVar + 2 * 2) * nrows_ + i] - (x[1 + (0) * nrows_ + i] + config_.vehicle.rear_hang_length * cos(x[1 + (2) * nrows_ + i]) - config_.vehicle.width / 2 * sin(x[1 + (2) * nrows_ + i])), 2)
    //                    + pow(x[1 + (NVar + 2 * 2 + 1) * nrows_ + i] - (x[1 + (1) * nrows_ + i] + config_.vehicle.rear_hang_length * sin(x[1 + (2) * nrows_ + i]) + config_.vehicle.width / 2 * cos(x[1 + (2) * nrows_ + i])), 2)    
    //                    + pow(x[1 + (NVar + 3 * 2) * nrows_ + i] - (x[1 + (0) * nrows_ + i] + config_.vehicle.rear_hang_length * cos(x[1 + (2) * nrows_ + i]) + config_.vehicle.width / 2 * sin(x[1 + (2) * nrows_ + i])), 2)         
    //                    + pow(x[1 + (NVar + 3 * 2 + 1) * nrows_ + i] - (x[1 + (1) * nrows_ + i] + config_.vehicle.rear_hang_length * sin(x[1 + (2) * nrows_ + i]) - config_.vehicle.width / 2 * cos(x[1 + (2) * nrows_ + i])), 2); 
    // }

    // double offset_ = hypot(-18-(-20), -21.0083668325-(-20));
    // double re_phi = atan2(21.0083668325-20, 20-18);
    // T infeasibility_temp = 0.0;
    // for(int i = 1; i < nrows_; i++) {
    //   infeasibility_temp += pow((x[1 + (0) * nrows_ + i] + offset_ * cos(x[1 + (2) * nrows_ + i] - re_phi)) - (x[1 + (0) * nrows_ + i - 1] + offset_ * cos(x[1 + (2) * nrows_ + i - 1] - re_phi)), 2) +
    //       pow((x[1 + (1) * nrows_ + i] + offset_ * sin(x[1 + (2) * nrows_ + i] - re_phi)) - (x[1 + (1) * nrows_ + i - 1] + offset_ * sin(x[1 + (2) * nrows_ + i - 1] - re_phi)), 2) / (dt * dt)
    //       - pow(config_.vehicle.max_velocity, 2);
    //         // infeasibility += pow(pow((x[1 + (0) * nrows_ + i] + offset_ * cos(x[1 + (2) * nrows_ + i] - re_phi)) - (x[1 + (0) * nrows_ + i - 1] + offset_ * cos(x[1 + (2) * nrows_ + i - 1] - re_phi)), 2) +
    //         // pow((x[1 + (1) * nrows_ + i] + offset_ * sin(x[1 + (2) * nrows_ + i] - re_phi)) - (x[1 + (1) * nrows_ + i - 1] + offset_ * sin(x[1 + (2) * nrows_ + i - 1] - re_phi)), 2) / (dt * dt)
    //         // - pow(config_.vehicle.max_velocity, 2), 2);
    //         // infeasibility += 100;
    //   }
      // if (pow(x[1 + (2) * nrows_ + i] - x[1 + (2) * nrows_ + i - 1], 2) / (dt * dt) > pow(config_.vehicle.omega_max,2)) {
      //   infeasibility += pow(pow(x[1 + (2) * nrows_ + i] - x[1 + (2) * nrows_ + i - 1], 2) / (dt * dt) - pow(config_.vehicle.omega_max,2), 2);
      // }
      // T distance_diff1 = x_diff_car1 * x_diff_car1 + y_diff_car1 * y_diff_car1;
      // T angle_diff1 = atan2(y_diff_car1, x_diff_car1);
      // T x_diff_car2 = x[1 + (0) * nrows_ + i - 1] + offset_ * cos(x[1 + (2) * nrows_ + i - 1] - re_phi) - x[1 + (0) * nrows_ + i - 2] + offset_ * cos(x[1 + (2) * nrows_ + i - 2] - re_phi);
      // T y_diff_car2 = x[1 + (1) * nrows_ + i - 1] + offset_ * sin(x[1 + (2) * nrows_ + i - 1] - re_phi) - x[1 + (1) * nrows_ + i - 2] + offset_ * sin(x[1 + (2) * nrows_ + i - 2] - re_phi);
      // T distance_diff2 = x_diff_car2 * x_diff_car2 + y_diff_car2 * y_diff_car2;
      // T angle_diff2 = atan2(y_diff_car2, x_diff_car2);
      // T a_diff = (distance_diff1 - distance_diff2) / (dtemp * dtemp);
      // T w_diff = (angle_diff1 - angle_diff2) / dtemp;
      // obj_value += config_.opti_w_diff_drive * (a_diff * a_diff + w_diff * w_diff);
    // }

    return infeasibility;
  }

  template<class T> bool eval_obj(Ipopt::Index n, const T *x, T& obj_value) {
    obj_value = config_.opti_t * x[0];
    T dt = x[0] / nfe_;
    // T dtemp = x[0] / nfe_;
    // T offset_ = hypot(-18-(-20), -21.0083668325-(-20));
    // T re_phi = atan2(21.0083668325-20, 20-18);
    // for(int i = 2; i < nrows_; i++) {
    //   T x_diff_car1 = x[1 + (0) * nrows_ + i] + offset_ * cos(x[1 + (2) * nrows_ + i] - re_phi) - x[1 + (0) * nrows_ + i - 1] + offset_ * cos(x[1 + (2) * nrows_ + i - 1] - re_phi);
    //   T y_diff_car1 = x[1 + (1) * nrows_ + i] + offset_ * sin(x[1 + (2) * nrows_ + i] - re_phi) - x[1 + (1) * nrows_ + i - 1] + offset_ * sin(x[1 + (2) * nrows_ + i - 1] - re_phi);
    //   T distance_diff1 = x_diff_car1 * x_diff_car1 + y_diff_car1 * y_diff_car1;
    //   T angle_diff1 = atan2(y_diff_car1, x_diff_car1);
    //   T x_diff_car2 = x[1 + (0) * nrows_ + i - 1] + offset_ * cos(x[1 + (2) * nrows_ + i - 1] - re_phi) - x[1 + (0) * nrows_ + i - 2] + offset_ * cos(x[1 + (2) * nrows_ + i - 2] - re_phi);
    //   T y_diff_car2 = x[1 + (1) * nrows_ + i - 1] + offset_ * sin(x[1 + (2) * nrows_ + i - 1] - re_phi) - x[1 + (1) * nrows_ + i - 2] + offset_ * sin(x[1 + (2) * nrows_ + i - 2] - re_phi);
    //   T distance_diff2 = x_diff_car2 * x_diff_car2 + y_diff_car2 * y_diff_car2;
    //   T angle_diff2 = atan2(y_diff_car2, x_diff_car2);
    //   T a_diff = (distance_diff1 - distance_diff2) / (dtemp * dtemp);
    //   T w_diff = (angle_diff1 - angle_diff2) / dtemp;
    //   obj_value += config_.opti_w_diff_drive * (a_diff * a_diff + w_diff * w_diff);
    // }
    for(int i = 0; i < nrows_; i++) {
      // a^2 + w^2
      // obj_value += config_.opti_w_a * x[1 + 5 * nrows_ + i] * x[1 + 5 * nrows_ + i]
      //     + config_.opti_w_omega * x[1 + 6 * nrows_ + i] * x[1 + 6 * nrows_ + i];
      obj_value += config_.opti_w_diff_drive * x[1 + 5 * nrows_ + i] * x[1 + 5 * nrows_ + i]
          + config_.opti_w_diff_drive * x[1 + 6 * nrows_ + i] * x[1 + 6 * nrows_ + i];
          // + config_.opti_w_phi * x[1 + 4 * nrows_ + i] * x[1 + 4 * nrows_ + i];
    }
    double offset_ = hypot(-18-(-20), -21.0083668325-(-20));
    double re_phi = atan2(21.0083668325-20, 20-18);
    for(int i = 1; i < nrows_; i++) {
      obj_value += config_.opti_w_diff_drive * x[1 + 3 * nrows_ + i] * x[1 + 3 * nrows_ + i];

      // obj_value += config_.opti_w_diff_drive*(pow((x[1 + (0) * nrows_ + i] + offset_ * cos(x[1 + (2) * nrows_ + i] - re_phi)) - (x[1 + (0) * nrows_ + i - 1] + offset_ * cos(x[1 + (2) * nrows_ + i - 1] - re_phi)), 2) +
      //     pow((x[1 + (1) * nrows_ + i] + offset_ * sin(x[1 + (2) * nrows_ + i] - re_phi)) - (x[1 + (1) * nrows_ + i - 1] + offset_ * sin(x[1 + (2) * nrows_ + i - 1] - re_phi)), 2) / (dt * dt));
            
            // infeasibility += pow(pow((x[1 + (0) * nrows_ + i] + offset_ * cos(x[1 + (2) * nrows_ + i] - re_phi)) - (x[1 + (0) * nrows_ + i - 1] + offset_ * cos(x[1 + (2) * nrows_ + i - 1] - re_phi)), 2) +
            // pow((x[1 + (1) * nrows_ + i] + offset_ * sin(x[1 + (2) * nrows_ + i] - re_phi)) - (x[1 + (1) * nrows_ + i - 1] + offset_ * sin(x[1 + (2) * nrows_ + i - 1] - re_phi)), 2) / (dt * dt)
            // - pow(config_.vehicle.max_velocity, 2), 2);
            // infeasibility += 100;
      }

    obj_value += w_inf_ * eval_infeasibility(x);
    return true;
  }

  template<class T> bool eval_constraints(Ipopt::Index n, const T *x, Ipopt::Index m, T *g) {
    return true;
  }

  virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value) {
    eval_obj(n, x, obj_value);
    return true;
  }

  virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f) {
    gradient(tag_f, n, x, grad_f);
    return true;
  }

  virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g) {
    eval_constraints(n, x, m, g);
    return true;
  }

  virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                          Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index *jCol,
                          Ipopt::Number* values) {
    if(values == nullptr) {
      // return the structure of the jacobian
      for(Ipopt::Index idx = 0; idx < nnz_jac; idx++) {
        iRow[idx] = rind_g[idx];
        jCol[idx] = cind_g[idx];
      }
    } else {
      // return the values of the jacobian of the constraints
      sparse_jac(tag_g, m, n, 1, x, &nnz_jac, &rind_g, &cind_g, &jacval, options_g);

      for(Ipopt::Index idx = 0; idx < nnz_jac; idx++) {
        values[idx] = jacval[idx];
      }
    }
    return true;
  }

  virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                      Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                      bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                      Ipopt::Index* jCol, Ipopt::Number* values) {
    if(values == nullptr) {
      // return the structure. This is a symmetric matrix, fill the lower left triangle only.
      for(Ipopt::Index idx = 0; idx < nnz_L; idx++) {
        iRow[idx] = rind_L[idx];
        jCol[idx] = cind_L[idx];
      }
    } else {
      // return the values. This is a symmetric matrix, fill the lower left triangle only
      obj_lam[0] = obj_factor;
      for(Ipopt::Index idx = 0; idx < m; idx++)
        obj_lam[1 + idx] = lambda[idx];

      set_param_vec(tag_L, m + 1, obj_lam);
      sparse_hess(tag_L, n, 1, const_cast<double *>(x), &nnz_L, &rind_L, &cind_L, &hessval, options_L);

      for(Ipopt::Index idx = 0; idx < nnz_L; idx++) {
        values[idx] = hessval[idx];
      }
    }

    return true;
  }

  virtual void finalize_solution(Ipopt::SolverReturn status,
                                 Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U,
                                 Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda,
                                 Ipopt::Number obj_value,
                                 const Ipopt::IpoptData* ip_data,
                                 Ipopt::IpoptCalculatedQuantities* ip_cq) {
    std::copy(x, x + n, result_.data());

    delete[] obj_lam;
    free(rind_g);
    free(cind_g);
    free(rind_L);
    free(cind_L);
    free(jacval);
    free(hessval);
  }


  /** Method to generate the required tapes */
  virtual void generate_tapes(Ipopt::Index n, Ipopt::Index m, Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag) {
    Ipopt::Number *lamp  = new double[m];
    Ipopt::Number *zl    = new double[m];
    Ipopt::Number *zu    = new double[m];

    adouble *xa   = new adouble[n];
    adouble *g    = new adouble[m];
    double *lam   = new double[m];
    double sig;
    adouble obj_value;

    double dummy;
    obj_lam   = new double[m+1];
    get_starting_point(n, true, x0_.data(), false, zl, zu, m, false, lamp);

    trace_on(tag_f);

    for(Ipopt::Index idx=0;idx<n;idx++)
      xa[idx] <<= x0_[idx];

    eval_obj(n,xa,obj_value);

    obj_value >>= dummy;

    trace_off();

    trace_on(tag_g);

    for(Ipopt::Index idx=0;idx<n;idx++)
      xa[idx] <<= x0_[idx];

    eval_constraints(n,xa,m,g);


    for(Ipopt::Index idx=0;idx<m;idx++)
      g[idx] >>= dummy;

    trace_off();

    trace_on(tag_L);

    for(Ipopt::Index idx=0;idx<n;idx++)
      xa[idx] <<= x0_[idx];
    for(Ipopt::Index idx=0;idx<m;idx++)
      lam[idx] = 1.0;
    sig = 1.0;

    eval_obj(n,xa,obj_value);

    obj_value *= mkparam(sig);
    eval_constraints(n,xa,m,g);

    for(Ipopt::Index idx=0;idx<m;idx++)
      obj_value += g[idx]*mkparam(lam[idx]);

    obj_value >>= dummy;

    trace_off();

    rind_g = nullptr;
    cind_g = nullptr;
    rind_L = nullptr;
    cind_L = nullptr;

    options_g[0] = 0;          /* sparsity pattern by index domains (default) */
    options_g[1] = 0;          /*                         safe mode (default) */
    options_g[2] = 0;
    options_g[3] = 0;          /*                column compression (default) */

    jacval = nullptr;
    hessval = nullptr;
    sparse_jac(tag_g, m, n, 0, x0_.data(), &nnz_jac, &rind_g, &cind_g, &jacval, options_g);

    nnz_jac_g = nnz_jac;

    options_L[0] = 0;
    options_L[1] = 1;

    sparse_hess(tag_L, n, 0, x0_.data(), &nnz_L, &rind_L, &cind_L, &hessval, options_L);
    nnz_h_lag = nnz_L;

    delete[] lam;
    delete[] g;
    delete[] xa;
    delete[] zu;
    delete[] zl;
    delete[] lamp;
  }

  std::vector<double> result_;

private:
  double w_inf_;
  int nfe_, vert_nvar_;
  int nvar_, nrows_, ncols_;
  const Constraints &profile_;
  const FullStates &guess_;
  const PlannerConfig &config_;

  std::vector<double> x0_;

  double *obj_lam;

  //** variables for sparsity exploitation
  unsigned int *rind_g;        /* row indices    */
  unsigned int *cind_g;        /* column indices */
  double *jacval;              /* values         */
  unsigned int *rind_L;        /* row indices    */
  unsigned int *cind_L;        /* column indices */
  double *hessval;             /* values */
  int nnz_jac;
  int nnz_L;
  int options_g[4];
  int options_L[4];
};

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
class hmfpcIPOPTInterface_diff_drive : public Ipopt::TNLP
{
public:
  hmfpcIPOPTInterface_diff_drive(double w_inf, const Constraints &profile, const FullStates &guess, const FullStates &pre_sol, const PlannerConfig &config)
    : w_inf_(w_inf), profile_(profile), guess_(guess), pre_sol_(pre_sol), config_(config) {
    nfe_ = guess_.states.size();
    nrows_ = nfe_ - 2;
    // disc_nvar_ = config_.vehicle.n_disc * 2;
    ncols_ = NVar; // 7

    // tf + [x y theta v phi a omega x_disc_0 y_disc_0 ... x_disc_n y_disc_n] (nfe * (7 + n * 2)) + theta_end
    nvar_ = 1 + nrows_ * ncols_ + 1;
    result_.resize(nvar_);
    x0_.resize(nvar_);
  }

  virtual ~hmfpcIPOPTInterface_diff_drive() = default;

  hmfpcIPOPTInterface_diff_drive(const hmfpcIPOPTInterface_diff_drive&) = delete;
  hmfpcIPOPTInterface_diff_drive& operator=(const hmfpcIPOPTInterface_diff_drive&) = delete;

  virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                            Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style) {
    n = nvar_;
    m = 0;
    generate_tapes(n, m, nnz_jac_g, nnz_h_lag);
    // use the C style indexing (0-based)
    index_style = C_STYLE;
    return true;
  }

  virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                               Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u) {
    Eigen::Map<Eigen::ArrayXXd> lb_mat(x_l + 1, nrows_, ncols_);
    x_l[0] = guess_.tf; // tf > 0.1
    x_u[0] = guess_.tf;
    // x_l[0] = 0.1; // tf > 0.1
    // x_u[0] = inf;
    x_l[nvar_ - 1] = -inf;
    x_u[nvar_ - 1] = inf;

    TrajectoryPointVector lb_vec;
    // car-like
    // lb_vec << -inf, -inf, -inf,
    //     config_.vehicle.min_velocity,
    //     -config_.vehicle.phi_min,
    //     -config_.vehicle.max_acceleration,
    //     -config_.vehicle.omega_max;
    lb_vec << -inf, -inf, -inf,
        config_.vehicle.min_vel_diff,
        -config_.vehicle.omg_acc_diff,
        -config_.vehicle.max_acc_diff,
        -config_.vehicle.omg_max_diff;
    lb_mat.leftCols<NVar>() = lb_vec.transpose().replicate(nrows_, 1);
    // lb_mat.rightCols(disc_nvar_) = profile_.corridor_lb.middleRows(1, nrows_);

    Eigen::Map<Eigen::ArrayXXd> ub_mat(x_u + 1, nrows_, ncols_);

    TrajectoryPointVector ub_vec;
    // car-like
    // ub_vec << inf, inf, inf,
    //     config_.vehicle.max_velocity,
    //     config_.vehicle.phi_max,
    //     config_.vehicle.max_acceleration,
    //     config_.vehicle.omega_max;
    ub_vec << inf, inf, inf,
        config_.vehicle.max_vel_diff,
        config_.vehicle.omg_acc_diff,
        config_.vehicle.max_acc_diff,
        config_.vehicle.omg_max_diff;
    ub_mat.leftCols<NVar>() = ub_vec.transpose().replicate(nrows_, 1);
    // ub_mat.rightCols(disc_nvar_) = profile_.corridor_ub.middleRows(1, nrows_);
    return true;
  }

  virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                  bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                  Ipopt::Index m, bool init_lambda,
                                  Ipopt::Number* lambda) {
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);

    Eigen::Map<Eigen::ArrayXXd> x0_mat(x + 1, nrows_, ncols_);
    auto x0_state_mat = x0_mat.leftCols<NVar>();
    // auto x0_disc_mat = x0_mat.rightCols(disc_nvar_);
    if (pre_sol_.states.empty()) {
      x[0] = guess_.tf;
      x[nvar_ - 1] = guess_.states[guess_.states.size() - 1].theta;

      for(int i = 0; i < nrows_; i++) {
        x0_state_mat.row(i) = guess_.states[i+1].vec();      
      }
    }
    else {
      x[0] = pre_sol_.tf;
      x[nvar_ - 1] = pre_sol_.states[pre_sol_.states.size() - 1].theta;

      for(int i = 0; i < nrows_; i++) {
        x0_state_mat.row(i) = pre_sol_.states[i+1].vec();
      }
      // auto x0_disc = config_.vehicle.GetDiscPositions(guess_.states[i+1].x, guess_.states[i+1].y, guess_.states[i+1].theta);
      // x0_disc_mat.row(i) = Eigen::Map<Eigen::VectorXd>(x0_disc.data(), x0_disc.size());
    }

    return true;
  }

  template<class T> T eval_infeasibility(const T *x) {
    T dt = x[0] / nfe_;
    T infeasibility = 0.0;
    infeasibility += pow(x[1 + (0) * nrows_ + 0] - guess_.states[0].x - dt * guess_.states[0].v * cos(guess_.states[0].theta), 2)
                     + pow(x[1 + (1) * nrows_ + 0] - guess_.states[0].y - dt * guess_.states[0].v * sin(guess_.states[0].theta), 2)
                     + pow(x[1 + (2) * nrows_ + 0] - guess_.states[0].theta - dt * guess_.states[0].omega, 2)
                    //  + pow(x[1 + (2) * nrows_ + 0] - guess_.states[0].theta - dt * guess_.states[0].v * tan(guess_.states[0].phi) / config_.vehicle.wheel_base, 2)
                     + pow(x[1 + (3) * nrows_ + 0] - guess_.states[0].v - dt * guess_.states[0].a, 2)
                     + pow(x[1 + (6) * nrows_ + 0] - guess_.states[0].omega - dt * guess_.states[0].phi, 2);

    for(int i = 1; i < nrows_; i++) {
      infeasibility += pow(x[1 + (0) * nrows_ + i] - x[1 + (0) * nrows_ + i-1] - dt * x[1 + (3) * nrows_ + i-1] * cos(x[1 + (2) * nrows_ + i-1]), 2)
                       + pow(x[1 + (1) * nrows_ + i] - x[1 + (1) * nrows_ + i-1] - dt * x[1 + (3) * nrows_ + i-1] * sin(x[1 + (2) * nrows_ + i-1]), 2)
                       + pow(x[1 + (2) * nrows_ + i] - x[1 + (2) * nrows_ + i-1] - dt * x[1 + (6) * nrows_ + i-1], 2)
                      //  + pow(x[1 + (2) * nrows_ + i] - x[1 + (2) * nrows_ + i-1] - dt * x[1 + (3) * nrows_ + i-1] * tan(x[1 + (4) * nrows_ + i-1]) / config_.vehicle.wheel_base, 2)
                       + pow(x[1 + (3) * nrows_ + i] - x[1 + (3) * nrows_ + i-1] - dt * x[1 + (5) * nrows_ + i-1], 2)
                       + pow(x[1 + (6) * nrows_ + i] - x[1 + (6) * nrows_ + i-1] - dt * x[1 + (4) * nrows_ + i-1], 2);
    }

    infeasibility += pow(guess_.states[guess_.states.size() - 1].x - x[1 + (0) * nrows_ + nrows_-1] - dt * x[1 + (3) * nrows_ + nrows_-1] * cos(x[1 + (2) * nrows_ + nrows_-1]), 2)
                     + pow(guess_.states[guess_.states.size() - 1].y - x[1 + (1) * nrows_ + nrows_-1] - dt * x[1 + (3) * nrows_ + nrows_-1] * sin(x[1 + (2) * nrows_ + nrows_-1]), 2)
                     + pow(x[nvar_ - 1] - x[1 + (2) * nrows_ + nrows_-1] - dt * x[1 + (6) * nrows_ + nrows_-1], 2)
                     + pow(guess_.states[guess_.states.size() - 1].v - x[1 + (3) * nrows_ + nrows_-1] - dt * x[1 + (5) * nrows_ + nrows_-1], 2)
                     + pow(guess_.states[guess_.states.size() - 1].omega - x[1 + (6) * nrows_ + nrows_-1] - dt * x[1 + (4) * nrows_ + nrows_-1], 2);
                    //  + pow(guess_.states[guess_.states.size() - 1].phi - x[1 + (4) * nrows_ + nrows_-1] - dt * x[1 + (6) * nrows_ + nrows_-1], 2);

    infeasibility += pow(sin(guess_.states[guess_.states.size() - 1].theta) - sin(x[nvar_ - 1]), 2) + pow(cos(guess_.states[guess_.states.size() - 1].theta) - cos(x[nvar_ - 1]), 2);

    // for(int i = 0; i < nrows_; i++) {
    //   for (int j = 0; j < config_.vehicle.n_disc; j++) {
    //     infeasibility += pow(x[1 + (NVar + j * 2) * nrows_ + i] - x[1 + (0) * nrows_ + i] - config_.vehicle.disc_coefficients[j] * cos(x[1 + (2) * nrows_ + i]), 2)
    //                      + pow(x[1 + (NVar + j * 2 + 1) * nrows_ + i] - x[1 + (1) * nrows_ + i] - config_.vehicle.disc_coefficients[j] * sin(x[1 + (2) * nrows_ + i]), 2);
    //   }
    // }

    return infeasibility;
  }

  template<class T> bool eval_obj(Ipopt::Index n, const T *x, T& obj_value) {
    obj_value = x[0];
    // T dtemp = x[0] / nfe_;
    // T offset_ = hypot(-18-(-20), -21.0083668325-(-20));
    // T re_phi = atan2(21.0083668325-20, 20-18);
    // for(int i = 2; i < nrows_; i++) {
    //   T x_diff_car1 = x[1 + (0) * nrows_ + i] + offset_ * cos(x[1 + (2) * nrows_ + i] - re_phi) - x[1 + (0) * nrows_ + i - 1] + offset_ * cos(x[1 + (2) * nrows_ + i - 1] - re_phi);
    //   T y_diff_car1 = x[1 + (1) * nrows_ + i] + offset_ * sin(x[1 + (2) * nrows_ + i] - re_phi) - x[1 + (1) * nrows_ + i - 1] + offset_ * sin(x[1 + (2) * nrows_ + i - 1] - re_phi);
    //   T distance_diff1 = x_diff_car1 * x_diff_car1 + y_diff_car1 * y_diff_car1;
    //   T angle_diff1 = atan2(y_diff_car1, x_diff_car1);
    //   T x_diff_car2 = x[1 + (0) * nrows_ + i - 1] + offset_ * cos(x[1 + (2) * nrows_ + i - 1] - re_phi) - x[1 + (0) * nrows_ + i - 2] + offset_ * cos(x[1 + (2) * nrows_ + i - 2] - re_phi);
    //   T y_diff_car2 = x[1 + (1) * nrows_ + i - 1] + offset_ * sin(x[1 + (2) * nrows_ + i - 1] - re_phi) - x[1 + (1) * nrows_ + i - 2] + offset_ * sin(x[1 + (2) * nrows_ + i - 2] - re_phi);
    //   T distance_diff2 = x_diff_car2 * x_diff_car2 + y_diff_car2 * y_diff_car2;
    //   T angle_diff2 = atan2(y_diff_car2, x_diff_car2);
    //   T a_diff = (distance_diff1 - distance_diff2) / (dtemp * dtemp);
    //   T w_diff = (angle_diff1 - angle_diff2) / dtemp;
    //   obj_value += config_.opti_w_diff_drive * (a_diff * a_diff + w_diff * w_diff);
    // }
    for(int i = 0; i < nrows_; i++) {
      obj_value += config_.opti_w_err * pow(x[1 + 0 * nrows_ + i] - guess_.states[i].x, 2)
          + config_.opti_w_err * pow(x[1 + 1 * nrows_ + i] - guess_.states[i].y, 2);
          // + config_.opti_w_theta * pow(sin(x[1 + 2 * nrows_ + i]) - sin(guess_.states[i].theta), 2)
          // + config_.opti_w_theta * pow(cos(x[1 + 2 * nrows_ + i]) - cos(guess_.states[i].theta), 2);
      // obj_value += config_.opti_w_a * x[1 + 3 * nrows_ + i] * x[1 + 3 * nrows_ + i]
      //     + config_.opti_w_omega * x[1 + 6 * nrows_ + i] * x[1 + 6 * nrows_ + i];
          // + config_.opti_w_omega * x[1 + 4 * nrows_ + i] * x[1 + 4 * nrows_ + i];
    }

    obj_value += w_inf_ * eval_infeasibility(x);
    return true;
  }

  template<class T> bool eval_constraints(Ipopt::Index n, const T *x, Ipopt::Index m, T *g) {
    return true;
  }

  virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value) {
    eval_obj(n, x, obj_value);
    return true;
  }

  virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f) {
    gradient(tag_f, n, x, grad_f);
    return true;
  }

  virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g) {
    eval_constraints(n, x, m, g);
    return true;
  }

  virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                          Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index *jCol,
                          Ipopt::Number* values) {
    if(values == nullptr) {
      // return the structure of the jacobian
      for(Ipopt::Index idx = 0; idx < nnz_jac; idx++) {
        iRow[idx] = rind_g[idx];
        jCol[idx] = cind_g[idx];
      }
    } else {
      // return the values of the jacobian of the constraints
      sparse_jac(tag_g, m, n, 1, x, &nnz_jac, &rind_g, &cind_g, &jacval, options_g);

      for(Ipopt::Index idx = 0; idx < nnz_jac; idx++) {
        values[idx] = jacval[idx];
      }
    }
    return true;
  }

  virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                      Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                      bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                      Ipopt::Index* jCol, Ipopt::Number* values) {
    if(values == nullptr) {
      // return the structure. This is a symmetric matrix, fill the lower left triangle only.
      for(Ipopt::Index idx = 0; idx < nnz_L; idx++) {
        iRow[idx] = rind_L[idx];
        jCol[idx] = cind_L[idx];
      }
    } else {
      // return the values. This is a symmetric matrix, fill the lower left triangle only
      obj_lam[0] = obj_factor;
      for(Ipopt::Index idx = 0; idx < m; idx++)
        obj_lam[1 + idx] = lambda[idx];

      set_param_vec(tag_L, m + 1, obj_lam);
      sparse_hess(tag_L, n, 1, const_cast<double *>(x), &nnz_L, &rind_L, &cind_L, &hessval, options_L);

      for(Ipopt::Index idx = 0; idx < nnz_L; idx++) {
        values[idx] = hessval[idx];
      }
    }

    return true;
  }

  virtual void finalize_solution(Ipopt::SolverReturn status,
                                 Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U,
                                 Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda,
                                 Ipopt::Number obj_value,
                                 const Ipopt::IpoptData* ip_data,
                                 Ipopt::IpoptCalculatedQuantities* ip_cq) {
    std::copy(x, x + n, result_.data());

    delete[] obj_lam;
    free(rind_g);
    free(cind_g);
    free(rind_L);
    free(cind_L);
    free(jacval);
    free(hessval);
  }


  /** Method to generate the required tapes */
  virtual void generate_tapes(Ipopt::Index n, Ipopt::Index m, Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag) {
    Ipopt::Number *lamp  = new double[m];
    Ipopt::Number *zl    = new double[m];
    Ipopt::Number *zu    = new double[m];

    adouble *xa   = new adouble[n];
    adouble *g    = new adouble[m];
    double *lam   = new double[m];
    double sig;
    adouble obj_value;

    double dummy;
    obj_lam   = new double[m+1];
    get_starting_point(n, true, x0_.data(), false, zl, zu, m, false, lamp);

    trace_on(tag_f);

    for(Ipopt::Index idx=0;idx<n;idx++)
      xa[idx] <<= x0_[idx];

    eval_obj(n,xa,obj_value);

    obj_value >>= dummy;

    trace_off();

    trace_on(tag_g);

    for(Ipopt::Index idx=0;idx<n;idx++)
      xa[idx] <<= x0_[idx];

    eval_constraints(n,xa,m,g);


    for(Ipopt::Index idx=0;idx<m;idx++)
      g[idx] >>= dummy;

    trace_off();

    trace_on(tag_L);

    for(Ipopt::Index idx=0;idx<n;idx++)
      xa[idx] <<= x0_[idx];
    for(Ipopt::Index idx=0;idx<m;idx++)
      lam[idx] = 1.0;
    sig = 1.0;

    eval_obj(n,xa,obj_value);

    obj_value *= mkparam(sig);
    eval_constraints(n,xa,m,g);

    for(Ipopt::Index idx=0;idx<m;idx++)
      obj_value += g[idx]*mkparam(lam[idx]);

    obj_value >>= dummy;

    trace_off();

    rind_g = nullptr;
    cind_g = nullptr;
    rind_L = nullptr;
    cind_L = nullptr;

    options_g[0] = 0;          /* sparsity pattern by index domains (default) */
    options_g[1] = 0;          /*                         safe mode (default) */
    options_g[2] = 0;
    options_g[3] = 0;          /*                column compression (default) */

    jacval = nullptr;
    hessval = nullptr;
    sparse_jac(tag_g, m, n, 0, x0_.data(), &nnz_jac, &rind_g, &cind_g, &jacval, options_g);

    nnz_jac_g = nnz_jac;

    options_L[0] = 0;
    options_L[1] = 1;

    sparse_hess(tag_L, n, 0, x0_.data(), &nnz_L, &rind_L, &cind_L, &hessval, options_L);
    nnz_h_lag = nnz_L;

    delete[] lam;
    delete[] g;
    delete[] xa;
    delete[] zu;
    delete[] zl;
    delete[] lamp;
  }

  std::vector<double> result_;

private:
  double w_inf_;
  int nfe_;
  int nvar_, nrows_, ncols_;
  const Constraints &profile_;
  const FullStates &guess_;
  const FullStates &pre_sol_;
  const PlannerConfig &config_;

  std::vector<double> x0_;

  double *obj_lam;

  //** variables for sparsity exploitation
  unsigned int *rind_g;        /* row indices    */
  unsigned int *cind_g;        /* column indices */
  double *jacval;              /* values         */
  unsigned int *rind_L;        /* row indices    */
  unsigned int *cind_L;        /* column indices */
  double *hessval;             /* values */
  int nnz_jac;
  int nnz_L;
  int options_g[4];
  int options_L[4];
};

class hmfpcIPOPTInterface_car_like_replan : public Ipopt::TNLP
{
public:
  hmfpcIPOPTInterface_car_like_replan(double w_inf, const Constraints &profile, const FullStates &guess, const FullStates &pre_sol, const PlannerConfig &config)
    : w_inf_(w_inf), profile_(profile), guess_(guess), pre_sol_(pre_sol), config_(config) {
    nfe_ = guess_.states.size();
    nrows_ = nfe_ - 2;
    // disc_nvar_ = config_.vehicle.n_disc * 2;
    ncols_ = NVar; // 7

    // tf + [x y theta v phi a omega x_disc_0 y_disc_0 ... x_disc_n y_disc_n] (nfe * (7 + n * 2)) + theta_end
    nvar_ = 1 + nrows_ * ncols_ + 1;
    result_.resize(nvar_);
    x0_.resize(nvar_);
  }

  virtual ~hmfpcIPOPTInterface_car_like_replan() = default;

  hmfpcIPOPTInterface_car_like_replan(const hmfpcIPOPTInterface_car_like_replan&) = delete;
  hmfpcIPOPTInterface_car_like_replan& operator=(const hmfpcIPOPTInterface_car_like_replan&) = delete;

  virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                            Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style) {
    n = nvar_;
    m = 0;
    generate_tapes(n, m, nnz_jac_g, nnz_h_lag);
    // use the C style indexing (0-based)
    index_style = C_STYLE;
    return true;
  }

  virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                               Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u) {
    Eigen::Map<Eigen::ArrayXXd> lb_mat(x_l + 1, nrows_, ncols_);
    x_l[0] = guess_.tf; // tf > 0.1
    x_u[0] = guess_.tf;
    // x_l[0] = 0.1; // tf > 0.1
    // x_u[0] = inf;
    x_l[nvar_ - 1] = -inf;
    x_u[nvar_ - 1] = inf;

    TrajectoryPointVector lb_vec;
    // car-like
    lb_vec << -inf, -inf, -inf,
        config_.vehicle.min_velocity,
        -config_.vehicle.phi_max,
        -config_.vehicle.max_acceleration,
        -config_.vehicle.omega_max;
    lb_mat.leftCols<NVar>() = lb_vec.transpose().replicate(nrows_, 1);
    // lb_mat.rightCols(disc_nvar_) = profile_.corridor_lb.middleRows(1, nrows_);

    Eigen::Map<Eigen::ArrayXXd> ub_mat(x_u + 1, nrows_, ncols_);

    TrajectoryPointVector ub_vec;
    // car-like
    ub_vec << inf, inf, inf,
        config_.vehicle.max_velocity,
        config_.vehicle.phi_max,
        config_.vehicle.max_acceleration,
        config_.vehicle.omega_max;
    ub_mat.leftCols<NVar>() = ub_vec.transpose().replicate(nrows_, 1);
    // ub_mat.rightCols(disc_nvar_) = profile_.corridor_ub.middleRows(1, nrows_);
    return true;
  }

  virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                  bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                  Ipopt::Index m, bool init_lambda,
                                  Ipopt::Number* lambda) {
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);

    Eigen::Map<Eigen::ArrayXXd> x0_mat(x + 1, nrows_, ncols_);
    auto x0_state_mat = x0_mat.leftCols<NVar>();
    // auto x0_disc_mat = x0_mat.rightCols(disc_nvar_);
    if (pre_sol_.states.empty()) {
      x[0] = guess_.tf;
      x[nvar_ - 1] = guess_.states[guess_.states.size() - 1].theta;

      for(int i = 0; i < nrows_; i++) {
        x0_state_mat.row(i) = guess_.states[i+1].vec();      
      }
    }
    else {
      x[0] = pre_sol_.tf;
      x[nvar_ - 1] = pre_sol_.states[pre_sol_.states.size() - 1].theta;

      for(int i = 0; i < nrows_; i++) {
        x0_state_mat.row(i) = pre_sol_.states[i+1].vec();
      }
      // auto x0_disc = config_.vehicle.GetDiscPositions(guess_.states[i+1].x, guess_.states[i+1].y, guess_.states[i+1].theta);
      // x0_disc_mat.row(i) = Eigen::Map<Eigen::VectorXd>(x0_disc.data(), x0_disc.size());
    }

    return true;
  }

  template<class T> T eval_infeasibility(const T *x) {
    T dt = x[0] / nfe_;

    T infeasibility = 0.0;
    infeasibility += pow(x[1 + (0) * nrows_ + 0] - guess_.states[0].x - dt * guess_.states[0].v * cos(guess_.states[0].theta), 2)
                     + pow(x[1 + (1) * nrows_ + 0] - guess_.states[0].y - dt * guess_.states[0].v * sin(guess_.states[0].theta), 2)
                    //  + pow(x[1 + (2) * nrows_ + 0] - guess_.states[0].theta - dt * guess_.states[0].omega, 2)
                     + pow(x[1 + (2) * nrows_ + 0] - guess_.states[0].theta - dt * guess_.states[0].v * tan(guess_.states[0].phi) / config_.vehicle.wheel_base, 2)
                     + pow(x[1 + (3) * nrows_ + 0] - guess_.states[0].v - dt * guess_.states[0].a, 2)
                     + pow(x[1 + (4) * nrows_ + 0] - guess_.states[0].phi - dt * guess_.states[0].omega, 2);

    for(int i = 1; i < nrows_; i++) {
      infeasibility += pow(x[1 + (0) * nrows_ + i] - x[1 + (0) * nrows_ + i-1] - dt * x[1 + (3) * nrows_ + i-1] * cos(x[1 + (2) * nrows_ + i-1]), 2)
                       + pow(x[1 + (1) * nrows_ + i] - x[1 + (1) * nrows_ + i-1] - dt * x[1 + (3) * nrows_ + i-1] * sin(x[1 + (2) * nrows_ + i-1]), 2)
                      //  + pow(x[1 + (2) * nrows_ + i] - x[1 + (2) * nrows_ + i-1] - dt * x[1 + (6) * nrows_ + i-1], 2)
                       + pow(x[1 + (2) * nrows_ + i] - x[1 + (2) * nrows_ + i-1] - dt * x[1 + (3) * nrows_ + i-1] * tan(x[1 + (4) * nrows_ + i-1]) / config_.vehicle.wheel_base, 2)
                       + pow(x[1 + (3) * nrows_ + i] - x[1 + (3) * nrows_ + i-1] - dt * x[1 + (5) * nrows_ + i-1], 2)
                       + pow(x[1 + (4) * nrows_ + i] - x[1 + (4) * nrows_ + i-1] - dt * x[1 + (6) * nrows_ + i-1], 2);
    }

    infeasibility += pow(guess_.states[guess_.states.size() - 1].x - x[1 + (0) * nrows_ + nrows_-1] - dt * x[1 + (3) * nrows_ + nrows_-1] * cos(x[1 + (2) * nrows_ + nrows_-1]), 2)
                     + pow(guess_.states[guess_.states.size() - 1].y - x[1 + (1) * nrows_ + nrows_-1] - dt * x[1 + (3) * nrows_ + nrows_-1] * sin(x[1 + (2) * nrows_ + nrows_-1]), 2)
                     + pow(x[nvar_ - 1] - x[1 + (2) * nrows_ + nrows_-1] - dt * x[1 + (6) * nrows_ + nrows_-1], 2)
                     + pow(guess_.states[guess_.states.size() - 1].v - x[1 + (3) * nrows_ + nrows_-1] - dt * x[1 + (5) * nrows_ + nrows_-1], 2)
                    //  + pow(guess_.states[guess_.states.size() - 1].omega - x[1 + (6) * nrows_ + nrows_-1] - dt * x[1 + (4) * nrows_ + nrows_-1], 2);
                     + pow(guess_.states[guess_.states.size() - 1].phi - x[1 + (4) * nrows_ + nrows_-1] - dt * x[1 + (6) * nrows_ + nrows_-1], 2);

    infeasibility += pow(sin(guess_.states[guess_.states.size() - 1].theta) - sin(x[nvar_ - 1]), 2) + pow(cos(guess_.states[guess_.states.size() - 1].theta) - cos(x[nvar_ - 1]), 2);
    // infeasibility += pow(guess_.states[guess_.states.size() - 1].theta - x[nvar_ - 1], 2);

    // for(int i = 0; i < nrows_; i++) {
    //   for (int j = 0; j < config_.vehicle.n_disc; j++) {
    //     infeasibility += pow(x[1 + (NVar + j * 2) * nrows_ + i] - x[1 + (0) * nrows_ + i] - config_.vehicle.disc_coefficients[j] * cos(x[1 + (2) * nrows_ + i]), 2)
    //                      + pow(x[1 + (NVar + j * 2 + 1) * nrows_ + i] - x[1 + (1) * nrows_ + i] - config_.vehicle.disc_coefficients[j] * sin(x[1 + (2) * nrows_ + i]), 2);
    //   }
    // }

    return infeasibility;
  }

  template<class T> bool eval_obj(Ipopt::Index n, const T *x, T& obj_value) {
    obj_value = x[0];
    // T dtemp = x[0] / nfe_;
    // T offset_ = hypot(-18-(-20), -21.0083668325-(-20));
    // T re_phi = atan2(21.0083668325-20, 20-18);
    // for(int i = 2; i < nrows_; i++) {
    //   T x_diff_car1 = x[1 + (0) * nrows_ + i] + offset_ * cos(x[1 + (2) * nrows_ + i] - re_phi) - x[1 + (0) * nrows_ + i - 1] + offset_ * cos(x[1 + (2) * nrows_ + i - 1] - re_phi);
    //   T y_diff_car1 = x[1 + (1) * nrows_ + i] + offset_ * sin(x[1 + (2) * nrows_ + i] - re_phi) - x[1 + (1) * nrows_ + i - 1] + offset_ * sin(x[1 + (2) * nrows_ + i - 1] - re_phi);
    //   T distance_diff1 = x_diff_car1 * x_diff_car1 + y_diff_car1 * y_diff_car1;
    //   T angle_diff1 = atan2(y_diff_car1, x_diff_car1);
    //   T x_diff_car2 = x[1 + (0) * nrows_ + i - 1] + offset_ * cos(x[1 + (2) * nrows_ + i - 1] - re_phi) - x[1 + (0) * nrows_ + i - 2] + offset_ * cos(x[1 + (2) * nrows_ + i - 2] - re_phi);
    //   T y_diff_car2 = x[1 + (1) * nrows_ + i - 1] + offset_ * sin(x[1 + (2) * nrows_ + i - 1] - re_phi) - x[1 + (1) * nrows_ + i - 2] + offset_ * sin(x[1 + (2) * nrows_ + i - 2] - re_phi);
    //   T distance_diff2 = x_diff_car2 * x_diff_car2 + y_diff_car2 * y_diff_car2;
    //   T angle_diff2 = atan2(y_diff_car2, x_diff_car2);
    //   T a_diff = (distance_diff1 - distance_diff2) / (dtemp * dtemp);
    //   T w_diff = (angle_diff1 - angle_diff2) / dtemp;
    //   obj_value += config_.opti_w_diff_drive * (a_diff * a_diff + w_diff * w_diff);
    // }
    for(int i = 0; i < nrows_; i++) {
      obj_value += config_.opti_w_x * pow(x[1 + 0 * nrows_ + i] - guess_.states[i].x, 2)
          + config_.opti_w_y * pow(x[1 + 1 * nrows_ + i] - guess_.states[i].y, 2);
          // + config_.opti_w_theta * pow(x[1 + 2 * nrows_ + i] - guess_.states[i].theta, 2);
          // + config_.opti_w_theta * pow(sin(x[1 + 2 * nrows_ + i]) - sin(guess_.states[i].theta), 2)
          // + config_.opti_w_theta * pow(cos(x[1 + 2 * nrows_ + i]) - cos(guess_.states[i].theta), 2);
      // obj_value += 
      //     // config_.opti_w_a * x[1 + 3 * nrows_ + i] * x[1 + 3 * nrows_ + i]
      //     config_.opti_w_omega * x[1 + 6 * nrows_ + i] * x[1 + 6 * nrows_ + i]
      //     + config_.opti_w_omega * x[1 + 5 * nrows_ + i] * x[1 + 5 * nrows_ + i];
    }

    obj_value += w_inf_ * eval_infeasibility(x);
    return true;
  }

  template<class T> bool eval_constraints(Ipopt::Index n, const T *x, Ipopt::Index m, T *g) {
    return true;
  }

  virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value) {
    eval_obj(n, x, obj_value);
    return true;
  }

  virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f) {
    gradient(tag_f, n, x, grad_f);
    return true;
  }

  virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g) {
    eval_constraints(n, x, m, g);
    return true;
  }

  virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                          Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index *jCol,
                          Ipopt::Number* values) {
    if(values == nullptr) {
      // return the structure of the jacobian
      for(Ipopt::Index idx = 0; idx < nnz_jac; idx++) {
        iRow[idx] = rind_g[idx];
        jCol[idx] = cind_g[idx];
      }
    } else {
      // return the values of the jacobian of the constraints
      sparse_jac(tag_g, m, n, 1, x, &nnz_jac, &rind_g, &cind_g, &jacval, options_g);

      for(Ipopt::Index idx = 0; idx < nnz_jac; idx++) {
        values[idx] = jacval[idx];
      }
    }
    return true;
  }

  virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                      Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                      bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                      Ipopt::Index* jCol, Ipopt::Number* values) {
    if(values == nullptr) {
      // return the structure. This is a symmetric matrix, fill the lower left triangle only.
      for(Ipopt::Index idx = 0; idx < nnz_L; idx++) {
        iRow[idx] = rind_L[idx];
        jCol[idx] = cind_L[idx];
      }
    } else {
      // return the values. This is a symmetric matrix, fill the lower left triangle only
      obj_lam[0] = obj_factor;
      for(Ipopt::Index idx = 0; idx < m; idx++)
        obj_lam[1 + idx] = lambda[idx];

      set_param_vec(tag_L, m + 1, obj_lam);
      sparse_hess(tag_L, n, 1, const_cast<double *>(x), &nnz_L, &rind_L, &cind_L, &hessval, options_L);

      for(Ipopt::Index idx = 0; idx < nnz_L; idx++) {
        values[idx] = hessval[idx];
      }
    }

    return true;
  }

  virtual void finalize_solution(Ipopt::SolverReturn status,
                                 Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U,
                                 Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda,
                                 Ipopt::Number obj_value,
                                 const Ipopt::IpoptData* ip_data,
                                 Ipopt::IpoptCalculatedQuantities* ip_cq) {
    std::copy(x, x + n, result_.data());

    delete[] obj_lam;
    free(rind_g);
    free(cind_g);
    free(rind_L);
    free(cind_L);
    free(jacval);
    free(hessval);
  }


  /** Method to generate the required tapes */
  virtual void generate_tapes(Ipopt::Index n, Ipopt::Index m, Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag) {
    Ipopt::Number *lamp  = new double[m];
    Ipopt::Number *zl    = new double[m];
    Ipopt::Number *zu    = new double[m];

    adouble *xa   = new adouble[n];
    adouble *g    = new adouble[m];
    double *lam   = new double[m];
    double sig;
    adouble obj_value;

    double dummy;
    obj_lam   = new double[m+1];
    get_starting_point(n, true, x0_.data(), false, zl, zu, m, false, lamp);

    trace_on(tag_f);

    for(Ipopt::Index idx=0;idx<n;idx++)
      xa[idx] <<= x0_[idx];

    eval_obj(n,xa,obj_value);

    obj_value >>= dummy;

    trace_off();

    trace_on(tag_g);

    for(Ipopt::Index idx=0;idx<n;idx++)
      xa[idx] <<= x0_[idx];

    eval_constraints(n,xa,m,g);


    for(Ipopt::Index idx=0;idx<m;idx++)
      g[idx] >>= dummy;

    trace_off();

    trace_on(tag_L);

    for(Ipopt::Index idx=0;idx<n;idx++)
      xa[idx] <<= x0_[idx];
    for(Ipopt::Index idx=0;idx<m;idx++)
      lam[idx] = 1.0;
    sig = 1.0;

    eval_obj(n,xa,obj_value);

    obj_value *= mkparam(sig);
    eval_constraints(n,xa,m,g);

    for(Ipopt::Index idx=0;idx<m;idx++)
      obj_value += g[idx]*mkparam(lam[idx]);

    obj_value >>= dummy;

    trace_off();

    rind_g = nullptr;
    cind_g = nullptr;
    rind_L = nullptr;
    cind_L = nullptr;

    options_g[0] = 0;          /* sparsity pattern by index domains (default) */
    options_g[1] = 0;          /*                         safe mode (default) */
    options_g[2] = 0;
    options_g[3] = 0;          /*                column compression (default) */

    jacval = nullptr;
    hessval = nullptr;
    sparse_jac(tag_g, m, n, 0, x0_.data(), &nnz_jac, &rind_g, &cind_g, &jacval, options_g);

    nnz_jac_g = nnz_jac;

    options_L[0] = 0;
    options_L[1] = 1;

    sparse_hess(tag_L, n, 0, x0_.data(), &nnz_L, &rind_L, &cind_L, &hessval, options_L);
    nnz_h_lag = nnz_L;

    delete[] lam;
    delete[] g;
    delete[] xa;
    delete[] zu;
    delete[] zl;
    delete[] lamp;
  }

  std::vector<double> result_;

private:
  double w_inf_;
  int nfe_;
  int nvar_, nrows_, ncols_;
  const Constraints &profile_;
  const FullStates &guess_;
  const FullStates &pre_sol_;
  const PlannerConfig &config_;

  std::vector<double> x0_;

  double *obj_lam;

  //** variables for sparsity exploitation
  unsigned int *rind_g;        /* row indices    */
  unsigned int *cind_g;        /* column indices */
  double *jacval;              /* values         */
  unsigned int *rind_L;        /* row indices    */
  unsigned int *cind_L;        /* column indices */
  double *hessval;             /* values */
  int nnz_jac;
  int nnz_L;
  int options_g[4];
  int options_L[4];
};

LightweightProblem::LightweightProblem(std::shared_ptr<PlannerConfig> config, std::shared_ptr<Environment> env)
    : IOptimizer(std::move(config), std::move(env)) {
  app_.Options()->SetIntegerValue("print_level", 0);
  app_.Options()->SetIntegerValue("max_iter", config_->opti_inner_iter_max);
  app_.Options()->SetNumericValue("bound_relax_factor", 0.0);
  app_.Options()->SetStringValue("linear_solver", "mumps");

  auto status = app_.Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    ROS_ERROR("solver initialization failed");
    exit(-1);
  }
}

bool LightweightProblem::Solve(
  double w_inf, const Constraints &profile, const FullStates &guess, FullStates &result, double &infeasibility, double& solve_time) {
  double solver_st = GetCurrentTimestamp();

  Ipopt::SmartPtr<hmfpcIPOPTInterface> interface = new hmfpcIPOPTInterface(w_inf, profile, guess, *config_);

  auto status = app_.OptimizeTNLP(interface);
  bool nlp_convergence = (status == Ipopt::Solve_Succeeded)
      || (status == Ipopt::Solved_To_Acceptable_Level)
      || (status == Ipopt::Feasible_Point_Found)
      || (status == Ipopt::Search_Direction_Becomes_Too_Small)
      || (status == Ipopt::Maximum_Iterations_Exceeded);
  if (!nlp_convergence) {
    return false;
  }

  result = ConvertVectorToStates(interface->result_.data(), guess.states.size(), profile.start, profile.goal);
  infeasibility = interface->eval_infeasibility(interface->result_.data());
  solve_time = GetCurrentTimestamp() - solver_st;

  std::cout << "car-like robot"
    << "wall_t: " << solve_time
    << ", status: " << status
    << ", factor: " << config_->opti_t
    << ", infeasibility: " << infeasibility
    << ", tf: " << result.tf << std::endl;

  return true;
}

bool LightweightProblem::Solve_diff_drive(
  double w_inf, const Constraints &profile, const FullStates &guess, const FullStates &pre_sol, FullStates &result, double &infeasibility, double& solve_time) {
  double solver_st = GetCurrentTimestamp();

  Ipopt::SmartPtr<hmfpcIPOPTInterface_diff_drive> interface = new hmfpcIPOPTInterface_diff_drive(w_inf, profile, guess, pre_sol, *config_);

  auto status = app_.OptimizeTNLP(interface);
  bool nlp_convergence = (status == Ipopt::Solve_Succeeded)
      || (status == Ipopt::Solved_To_Acceptable_Level)
      || (status == Ipopt::Feasible_Point_Found)
      || (status == Ipopt::Search_Direction_Becomes_Too_Small)
      || (status == Ipopt::Maximum_Iterations_Exceeded);
  if (!nlp_convergence) {
    return false;
  }

  result = ConvertVectorToStates(interface->result_.data(), guess.states.size(), guess.states[0], guess.states[guess.states.size() - 1]);
  infeasibility = interface->eval_infeasibility(interface->result_.data());
  solve_time = GetCurrentTimestamp() - solver_st;
  std::cout << "diff-drive robot"
    << "wall_t: " << solve_time
    << ", status: " << status
    << ", infeasibility: " << infeasibility
    << ", tf: " << result.tf << std::endl;
  double dis_error = 0.0, ori_error = 0.0;
  double dis_error_max = 0.0, ori_error_max = 0.0;
  for (int i = 0; i < result.states.size(); i++) {
    double dis_error_cur = hypot(result.states[i].x - guess.states[i].x, result.states[i].y - guess.states[i].y);
    double ori_error_cur = fabs(result.states[i].theta - guess.states[i].theta);
    dis_error_max = std::max(dis_error_max, dis_error_cur);
    ori_error_max = std::max(ori_error_max, ori_error_cur);
    dis_error += dis_error_cur;
    ori_error += ori_error_cur;
  }
  ROS_WARN("distance average error: %.6f, oritation average error: %.6f", dis_error / result.states.size(), ori_error / result.states.size());
  ROS_WARN("distance max error: %.6f, oritation max error: %.6f", dis_error_max, ori_error_max);
  return true;
}

bool LightweightProblem::Solve_replan(
  double w_inf, const Constraints &profile, const FullStates &guess, const FullStates &pre_sol, FullStates &result, double &infeasibility, double &solution_car_like) {
  double solver_st = GetCurrentTimestamp();

  Ipopt::SmartPtr<hmfpcIPOPTInterface_car_like_replan> interface = new hmfpcIPOPTInterface_car_like_replan(w_inf, profile, guess, pre_sol, *config_);

  auto status = app_.OptimizeTNLP(interface);
  bool nlp_convergence = (status == Ipopt::Solve_Succeeded)
      || (status == Ipopt::Solved_To_Acceptable_Level)
      || (status == Ipopt::Feasible_Point_Found)
      || (status == Ipopt::Search_Direction_Becomes_Too_Small)
      || (status == Ipopt::Maximum_Iterations_Exceeded);
  if (!nlp_convergence) {
    return false;
  }

  result = ConvertVectorToStates(interface->result_.data(), guess.states.size(), guess.states[0], guess.states[guess.states.size() - 1]);
  infeasibility = interface->eval_infeasibility(interface->result_.data());
  solution_car_like = GetCurrentTimestamp() - solver_st;

  std::cout << "car-like robot (replanned)"
    << "wall_t: " << solution_car_like
    << ", status: " << status
    << ", infeasibility: " << infeasibility
    << ", tf: " << result.tf << std::endl;
  double dis_error = 0.0, ori_error = 0.0;
  double dis_error_max = 0.0, ori_error_max = 0.0;
  for (int i = 0; i < result.states.size(); i++) {
    double dis_error_cur = hypot(result.states[i].x - guess.states[i].x, result.states[i].y - guess.states[i].y);
    double ori_error_cur = fabs(result.states[i].theta - guess.states[i].theta);
    dis_error_max = std::max(dis_error_max, dis_error_cur);
    ori_error_max = std::max(ori_error_max, ori_error_cur);
    dis_error += dis_error_cur;
    ori_error += ori_error_cur;
  }
  ROS_WARN("distance average error: %.6f, oritation average error: %.6f", dis_error / result.states.size(), ori_error / result.states.size());
  ROS_WARN("distance max error: %.6f, oritation max error: %.6f", dis_error_max, ori_error_max);
  return true;
}

bool LightweightProblem::SolveFm(
  double w_inf, const std::vector<Constraints> &profile, const std::vector<FullStates> &guess, std::vector<FullStates> &result, double &infeasibility, double& solve_time, std::vector<double>& height_cons) {
  double solver_st = GetCurrentTimestamp();

  Ipopt::SmartPtr<hmfpcIPOPTInterfaceFm> interface = new hmfpcIPOPTInterfaceFm(w_inf, profile, guess, *config_, height_cons);

  auto status = app_.OptimizeTNLP(interface);
  bool nlp_convergence = (status == Ipopt::Solve_Succeeded)
      || (status == Ipopt::Solved_To_Acceptable_Level)
      || (status == Ipopt::Feasible_Point_Found)
      || (status == Ipopt::Search_Direction_Becomes_Too_Small)
      || (status == Ipopt::Maximum_Iterations_Exceeded);
  if (!nlp_convergence) {
    return false;
  }
  result = ConvertVectorToJointStates(guess.size(), interface->result_.data(), guess[0].states.size(), profile);
  VVCM vvcm;
  double cost = 0.0;
  for (int i = 0; i < result.size(); i++) {
    cost += result[0].tf;
    for(int j = 0; j < result[i].states.size(); j++) {
      // a^2 + w^2
      cost += pow(result[i].states[j].a, 2) + pow(result[i].states[j].omega, 2);
          // + config_.opti_w_phi * x[1 + 4 * nrows_ + i] * x[1 + 4 * nrows_ + i];
      // cost += pow((pow(result[1].states[j].x - result[0].states[j].x, 2)  + pow(result[1].states[j].y - result[0].states[j].y, 2) ) - vvcm.xv2t * vvcm.xv2t, 2);
      // cost += pow((pow(result[2].states[j].x - result[1].states[j].x, 2)  + pow(result[2].states[j].y - result[1].states[j].y, 2) ) - vvcm.xv2t * vvcm.xv2t, 2);
      // cost += pow((pow(result[2].states[j].x - result[0].states[j].x, 2)  + pow(result[2].states[j].y - result[0].states[j].y, 2) ) - vvcm.xv2t * vvcm.xv2t, 2);
    }
  }
  infeasibility = interface->eval_infeasibility(interface->result_.data());
  solve_time = GetCurrentTimestamp() - solver_st;

  std::cout << "car-like robot"
    << "wall_t: " << solve_time
    << ", status: " << status
    << ", factor: " << config_->opti_t
    << ", infeasibility: " << infeasibility
    << ", tf: " << result[0].tf 
    << ", cost: " << cost << std::endl;

  return true;
}

}
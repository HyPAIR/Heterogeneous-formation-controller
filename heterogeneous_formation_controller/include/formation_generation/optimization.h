#ifndef MPC_H
#define MPC_H

#include <vector>
#include <eigen3/Eigen/Core>

#define DT 0.32 // time step duration dt in s 


// Set weights parameters for the cost function
#define W_X 0.05
#define W_Y 0.05
#define W_DV 1.0
#define W_DOMEGA 3.0
#define W_THETA 1000
#define W_FINAL 1000

// Set lower and upper limits for variables.
#define MAXPHI 0.69
#define MAXOMEGA 1.5 // 25 deg in rad, used as delta bound
#define MAXV  1 // Maximal a value
#define BOUND 1.0e20 // Bound value for other variables


using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state);
  void Solve();
  void Solve(int index);
  vector<double> mpc_x;
  vector<double> mpc_y;
};

#endif /* MPC_H */

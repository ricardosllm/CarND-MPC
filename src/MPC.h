#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  size_t N  = 25;
  double dt = 0.05;

  const double Lf = 2.67;

  int state_elems    = 6;
  int actuator_elems = 2;

  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state,
                       Eigen::VectorXd coeffs,
                       vector<vector<double>> &all_vars);
};

#endif /* MPC_H */

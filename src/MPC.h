#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// Use the Vector type for CPPAD library
typedef CPPAD_TESTVECTOR(double) Dvector;

class MPC
{
private:
  // Variable to keep track of whether the initialization of the MPC object
  // is pending or not
  bool is_initialized = false;

  // Private variables to keep track of the previous control applied
  double prev_delta;
  double prev_a;

  // Number of model variables
  size_t n_vars;

  // Number of constraints
  size_t n_constraints;

  // Vector for the variable upper and lower bounds for the state 'x'
  Dvector vars_lowerbound;
  Dvector vars_upperbound;

  // Initial value of the independent variables.
  Dvector vars;

  // Lower and upper limits for the state variable constraints
  Dvector constraints_lowerbound;
  Dvector constraints_upperbound;

  // Initialize function which sets up all the variable and constraints limits
  void Initialize(void);
public:
  // Constructor and Destructor
  MPC();
  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */

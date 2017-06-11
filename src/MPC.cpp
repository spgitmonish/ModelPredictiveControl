#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 25;
double dt = 0.05;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

class FG_eval
{
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;

  // Coefficients of the fitted polynomial are tied to the variable within the
  // class FG_eval
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  // Double vector from the CPPAD library
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  // `fg` is a vector containing the cost and constraints.
  // `vars` is a vector containing the variable values (state & actuators).
  // NOTE: This is for invoking () operator to pass in extra variables for
  //       the object is invoked with the passed in vectors
  void operator()(ADvector& fg, const ADvector& vars)
  {
    // TODO: Implement MPC
  }
};


// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

// Function takes in the initial state and coefficients of the fitting
// polynomial. This function mostly sets up the vehicle model constraints
// and variables for the Ipopt.
vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Initial state variables
  double x = x0[0];
  double y = x0[1];
  double psi = x0[2];
  double v = x0[3];
  double cte = x0[4];
  double epsi = x0[5];

  // TODO: Set the number of model variables (includes both states and inputs).

  // Set up the number of model variables (includes state & actuator inputs).
  // NOTE 1: The state is a 6 element vector, the actuators is a 2
  //       element vector and there are 'N' timesteps.
  //       The number of variables is: 6 * N + 2 * N-1.
  // NOTE 2: The number of actuations is N-1 when number of time steps is N
  size_t n_vars = 6 * N + 2 * (N - 1);

  // Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // NOTE: This should be 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++)
  {
    vars[i] = 0;
  }

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // Vector for the variable upper and lower bounds for the state 'x'
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // TODO: Set lower and upper limits for variables.

  // Set up all non-actuators variables upper and lowerlimits
  // to the max negative and positive values.
  // NOTE: Delta start is when the actuator variables within the vector start
  for (int i = 0; i < delta_start; i++)
  {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // TODO: Set the number of constraints
  // The upper and lower limits of delta are set to -25 and 25 degrees
  // NOTE: The values are in radians.
  for (int i = delta_start; i < a_start; i++)
  {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = a_start; i < n_vars; i++)
  {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the state variable constraints
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  // NOTE: All of these should be 0 except the initial state indices.
  for (int i = 0; i < n_constraints; i++)
  {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Initial state variable constraints lower bounds because of initial state
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  // Initial state variable constraints upper bounds because of initial state
  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // Object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // NOTE: You don't have to worry about these options
  // Options for IPOPT solver
  std::string options;
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // Place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound,
                                        vars_upperbound, constraints_lowerbound,
                                        constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // NOTE: {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return {solution.x[delta_start], solution.x[a_start]};
}

#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set the timestep length and duration
size_t N = 20;
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

// Reference velocity for the cost function calculation
double ref_v = 40;

// The solver takes all the state variables and actuator variables in single vector.
// Establish when one variable starts and ends within the single vector.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

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
    // Implement MPC
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // The part of the cost based on the reference state.
    // Here we minimize cross track, heading and velocity error
    for (int t = 0; t < N; t++)
    {
      fg[0] += CppAD::pow(vars[cte_start + t], 2);
      fg[0] += CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the magnitude of actuators and prevent sharp spikes i.e. change rate
    for (int t = 0; t < N - 1; t++)
    {
      fg[0] += CppAD::pow(vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations. Make control
    // decisions smoother, the next control should be similar to the current.
    // NOTE: By multiplying a constant the cost increases with a bigger difference
    //       between the values at time 't+1' and 't'
    for (int t = 0; t < N - 2; t++)
    {
      fg[0] += 15000 * (CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2));
      fg[0] += 10000 * (CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2));
    }

    // This section sets up the model constraints.

    // Initial model constraint is based on the initial state passed in.
    // NOTE: 1 is added to each of the starting indices due to cost being located
    //       at index 0 of `fg'.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints at t+1 are based on the vehicle model
    for (int t = 1; t < N; t++)
    {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      // Here we have a third order polynomial represting the reference points
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
      // The derivative of the polynomial is used to calculate the desired psi
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * pow(x0, 2));

      // The idea here is to constraint this value to be 0.
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);

      // NOTE: This is where the model constraints are set up for each timestep.
      //       Except for the initial model constraints  bounds being set to
      //       the maximum/minimum possible values in MPC::Solve(). The remaining
      //       constraints is bounded to 0. So the optimizer will force this value
      //       of fg for each time step 't+1' to be zero.
    }
  }
};


// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

// Initialize all the necessary variable and constraints limits
void MPC::Initialize()
{
  // Set up the number of model variables (includes state & actuator inputs).
  // NOTE 1: The state is a 6 element vector, the actuators is a 2
  //       element vector and there are 'N' timesteps.
  //       The number of variables is: 6 * N + 2 * N-1.
  // NOTE 2: The number of actuations is N-1 when number of time steps is N
  n_vars = 6 * N + 2 * (N - 1);

  // Set the size of the variables vector before initializing
  vars = Dvector(n_vars);

  // This should be 0 besides initial state.
  // NOTE: The initial state is setup fresh for every Solve() call
  for (int i = 0; i < n_vars; i++)
  {
    vars[i] = 0;
  }

  // Set the size of the variables upper and lower vector
  vars_lowerbound = Dvector(n_vars);
  vars_upperbound = Dvector(n_vars);

  // Set up all non-actuators variables upper and lowerlimits
  // to the max negative and positive values.
  // NOTE: Delta start is when the actuator variables within the vector start
  for (int i = 0; i < delta_start; i++)
  {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25 degrees
  for (int i = delta_start; i < a_start; i++)
  {
    vars_lowerbound[i] = -20;
    vars_upperbound[i] = 20;
  }

  // Acceleration/decceleration upper and lower limits.
  for (int i = a_start; i < n_vars; i++)
  {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Set the number of constraints
  n_constraints = N * 6;

  // Set the size of the constraints upper and lower vector
  constraints_lowerbound = Dvector(n_constraints);
  constraints_upperbound = Dvector(n_constraints);

  // All of these should be 0 except the initial state indices.
  // NOTE: The initial state is setup in the Solve() function for every iteration
  for (int i = 0; i < n_constraints; i++)
  {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Set the previous delta and accelaration to 0.0
  prev_delta = 0.0;
  prev_a = 0.0;

  // Set the initialization flag to true
  is_initialized = true;
}

// Function takes in the initial state and coefficients of the fitting
// polynomial. This function mostly sets up the vehicle model constraints
// and variables for the Ipopt.
vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Check if the MPC object has been initialized before
  if(!is_initialized)
  {
    Initialize();
  }

  // Initial state variables
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Account for the lag(100ms = 0.1s) in actuator execution
  // The next state after 100ms is calculated using the vehicle model
  double lag = 0.1;

  x = x + v * cos(psi) * lag;
  y = y + v * sin(psi) * lag;
  psi = psi + (v/Lf) * prev_delta * lag;
  v = v + prev_a * lag;
  cte = cte + v * sin(epsi) * lag;
  epsi = epsi + (v/Lf) * prev_delta * lag;

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

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

  // Store the first actuator values for the next iteration to account for the lag
  prev_delta = solution.x[delta_start];
  prev_a = solution.x[a_start];

  return {solution.x[delta_start], solution.x[a_start]};
}

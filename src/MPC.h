#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// Use the Vector type for CPPAD library
typedef CPPAD_TESTVECTOR(double) Dvector;

// Structure for storing the different configurations for speed
typedef struct mpc_speed_config
{
  // Number of timesteps
  size_t N;

  // Duration of timestep
  double dt;

  // Weights for different variables in the cost function
  double w_cte;
  double w_epsi;
  double w_v;
  double w_delta;
  double w_a;
  double w_delta_time;
  double w_a_time;
} mpc_speed_config;

// Objects of type mpc_speed_config with different configurations for different
// speeds, the speed is set during execution of the object file.
// This is the default config tuned for 40mph
const mpc_speed_config config_default = {
  20,     // Number of timesteps
  0.05,   // Duration of timestep
  1.0,    // Weight for cross track error cost element
  1.0,    // Weight for heading error cost element
  1.0,    // Weight velocity error cost element
  1.0,    // Weight for heading magnitude cost element
  1.0,    // Weight for acceleration magnitude cost element
  15000,  // Weight for heading magnitude between timesteps cost element
  10000,  // Weight for acceleration magnitude between timesteps cost element
};

// This is the 60mph config
const mpc_speed_config config_60_mph = {
  10,     // Number of timesteps
  0.05,   // Duration of timestep
  2.0,    // Weight for cross track error cost element
  2.0,    // Weight for heading error cost element
  1.0,    // Weight velocity error cost element
  2.0,    // Weight for heading magnitude cost element
  1.0,    // Weight for acceleration magnitude cost element
  15000,  // Weight for heading magnitude between timesteps cost element
  10000,  // Weight for acceleration magnitude between timesteps cost element
};

// This is the 80mph config
const mpc_speed_config config_80_mph = {
  10,     // Number of timesteps
  0.05,   // Duration of timestep
  2.0,    // Weight for cross track error cost element
  2.0,    // Weight for heading error cost element
  1.0,    // Weight velocity error cost element
  2.0,    // Weight for heading magnitude cost element
  0.5,    // Weight for acceleration magnitude cost element
  15000,  // Weight for heading magnitude between timesteps cost element
  1500,   // Weight for acceleration magnitude between timesteps cost element
};

// Extern for main.cpp to set the reference velocity
extern double ref_v;

// Structure for storing MPC output
typedef struct mpc_output
{
  // For storing the first steering angle and acceleration output from
  // the ipopt optimizer
  double steer_angle;
  double throttle;

  // Vector of predicted x & y values
  vector<double> pred_x_vals;
  vector<double> pred_y_vals;
} mpc_output;

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
  mpc_output Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */

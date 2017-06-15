// Flag for displaying debug
#define DEBUG 0

// Flag for enabling verbose debug
#define DEBUG_VERBOSE DEBUG && 0

// Structure to store the previous state of MPC
typedef struct previous_state
{
  // Previous steering angle and throttle output from MPC
  double steer_value;
  double throttle_value;

  // Previous predicted way points
  vector<double> mpc_x_vals;
  vector<double> mpc_y_vals;

  // Previous reference way points
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // Counter to keep track of when to run MPC
  int run_mpc_counter;
} previous_state;

// Function definitions
// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x)
{
  double result = 0.0;

  for (int i = 0; i < coeffs.size(); i++)
  {
    result += coeffs[i] * pow(x, i);
  }

  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++)
  {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++)
  {
    for (int i = 0; i < order; i++)
    {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

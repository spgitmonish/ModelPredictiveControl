// Set the timestep length and duration
// NOTE: These are set to default values
size_t N;
double dt;

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
double ref_v;

// The solver takes all the state variables and actuator variables in single vector.
// Establish when one variable starts and ends within the single vector.
size_t x_start;
size_t y_start;
size_t psi_start;
size_t v_start;
size_t cte_start;
size_t epsi_start;
size_t delta_start;
size_t a_start;

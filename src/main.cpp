#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "helper.h"

// For convenience
using json = nlohmann::json;

// Using CppAD library
using CppAD::AD;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");

  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Variable which stores the loaded config
extern mpc_speed_config loaded_config;

int main(int argc, char** argv)
{
  uWS::Hub h;

  // MPC object
  MPC mpc;

  // Assert if the number of arguments passed in is more than 2.
  // Including the there is only one additional argument, the velocity
  assert(argc<=2 && "Only 1(velocity) argument can be passed in!");

  if(argc == 1)
  {
    cout << "Default to reference velocity of 60 mph" << endl;

    // Set up the reference velocity
    ref_v = 60;
  }
  else
  {
    // Get the velocity argument and convert it into integer
    ref_v = atoi(argv[1]);

    // Print a message for the user
    if(ref_v > 80)
    {
      cout << "Reference velocity is higher than the model supports, default to 80mph" << endl;
    }
    else
    {
      cout << ref_v << "mph set as reference velocity" << endl;
    }
  }

  // Convert reference velocity into m/s because the latency is in seconds
  ref_v = ref_v * 0.447;

  // The configuration is loaded from a constant object of the same type
  loaded_config = config_default;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
  #if DEBUG_VERBOSE
    cout << sdata << endl;
  #endif
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
    {
      string s = hasData(sdata);
      if (s != "")
      {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          // Convert velocity to m/s
          v = v * 0.447;

          // Variables for setting steeting value and throttle
          double steer_value;
          double throttle_value;

          // Initial state
          Eigen::VectorXd state(6);

          // Polynomial coefficients
          Eigen::VectorXd coeffs;

          // Eigen vectors for storing the converted co-ordinates
          Eigen::VectorXd xvals(ptsx.size());
          Eigen::VectorXd yvals(ptsy.size());

          // Copy the data into Eigen vectors after converting to car-cordinates
          // NOTE: Great visualization in this link: http://bit.ly/2wxuApz
          for(size_t index = 0; index < ptsx.size(); index++)
          {
            xvals[index] = (ptsx[index] - px) * cos(psi) + (ptsy[index] - py) * sin(psi);
            yvals[index] = (ptsy[index] - py) * cos(psi) - (ptsx[index] - px) * sin(psi);
          }

          // Fit a 3rd order polynomial for the x, y data
          coeffs = polyfit(xvals, yvals, 3);

          // Using the coefficients of the polynomial find the cross track error
          // At the origin of the car co-ordinates (0, 0)
          // NOTE: Cross track error is calculated along the 'y' direction
          double cte = polyeval(coeffs, 0) - 0;

          // Calculate the orientation error
          // NOTE 1: The desired orientation is calculated as f'(x)
          //       The polynomial, f(x):
          //       coeffs[0] + coeffs[1] * x + coeffs[2] * x^2 + coeffs[3] * x^3
          //       The derivative, f'(x):
          //       coeffs[1] + 2 * coeffs[2] * x + 3 * coeffs[3] * x^2
          // NOTE 2: Due to psi starting at 0 the orientation error is -f'(x).
          // NOTE 3: Since the initial state is (0, 0) the only non-zero term
          //         remaining in f'(x) is coeffs[1]
          double epsi = -CppAD::atan(coeffs[1]);

          // Set up the state vector
          state << 0, 0, 0, v, cte, epsi;

          // Calculate the new state
          mpc_output vars = mpc.Solve(state, coeffs);

          // Set up the steering and throttle values
          // NOTE: Both are in between [-1, 1] for the simulator
          steer_value = vars.steer_angle;
          throttle_value = vars.throttle;

          json msgJson;
          // Divide by deg2rad(25) before sending the steering value back.
          // NOTE: Normalizing values between [-1, 1].
          msgJson["steering_angle"] = -steer_value/deg2rad(25);
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals = vars.pred_x_vals;
          vector<double> mpc_y_vals = vars.pred_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          // NOTE: The way point where the car is currently is not displayed because
          //       it leads to a reference line being drawn behind the car as well
          //       in the simulator
          vector<double> next_x_vals(xvals.data() + 1, xvals.data() + xvals.size() - 1);
          vector<double> next_y_vals(yvals.data() + 1, yvals.data() + yvals.size() - 1);

          //.. add (x,y) points to list here, points are in reference to the
          // vehicle's coordinate system the points in the simulator are
          // connected by a Yellow line
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
        #if DEBUG_VERBOSE
          std::cout << msg << std::endl;
        #endif

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
  {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
  {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}

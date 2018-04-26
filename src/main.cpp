#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "matplotlibcpp.h"


// for convenience
using json = nlohmann::json;
namespace plt = matplotlibcpp;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
std::vector<double> polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  double d_result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    d_result += i * coeffs[i] * pow(x, i-1);
  }
  return {result, d_result};
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;



  std::vector<double> steering_data;
  std::vector<double> acceleration_data;
  std::vector<double> cte_data;
  std::vector<double> epsi_data;
  std::vector<std::vector<double>> data_position(4);
  bool stop_flag = false;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc, &steer_data, &acceleration_data, &cte_data, &epsi_data, &data_position, &stop_flag](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode)  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

           v = v * 0.44704; // convert to ms
          // making  them static for state updates because of the  delay in the controller
          static double steer_value = 0.0;
          static double throttle_value = 0.0;


          // State in the Local frame.
          Eigen::VectorXd state(6);
          double x = 0.0;
          double y = 0.0;
          double psi_local = 0.0;
          // Convert the x and y points to local reference frame
          vector<double> x_refs = {};
          vector<double> y_refs = {};
          double cos_psi = std::cos(psi);
          double sin_psi = std::sin(psi);
          for (size_t i=0; i < ptsx.size(); ++i) {
            double x_local = cos_psi*(ptsx[i] - px) + sin_psi*(ptsy[i] - py);
            double y_local = cos_psi*(ptsy[i] - py) - sin_psi*(ptsx[i] - px);
            x_refs.push_back(x_local);
            y_refs.push_back(y_local);
          }
          // Lets fit the polynomial to update the  reference trajectory coordinates
          Eigen::VectorXd xpoints = Eigen::Map<Eigen::VectorXd>(x_refs.data(), x_refs.size());
          Eigen::VectorXd ypoints = Eigen::Map<Eigen::VectorXd>(y_refs.data(), y_refs.size());
          Eigen::VectorXd coeffs = polyfit(xpoints, ypoints, 2);
          double cte = -coeffs[0]; // since y = 0 in local coordinate frame
          double epsi = -std::atan(coeffs[1]); // since psi = 0 in local coordinate frame
          // On real time a delay may occur which could be because out of
          // the actuator, or the sensor latency in measuring the real world.
          // Because of delay we propagate state in time so that MPC initial condition is not the current one,
          // but the one after 100 milliseconds
          const int delay = 100;
          const double Lf = 2.67;
          const double dt = ((delay) ? delay/1000.0 : 0.0);
          double dx = v * std::cos(psi) * dt;
          double dy = v * std::sin(psi) * dt;
          // predict coordinates for the reference trajectory
          if (delay) {
              px += dx;
              py += dy;
              psi -= v/Lf * steer_value * deg2rad(25.0) * dt;
          }
          cos_psi = std::cos(psi);
          sin_psi = std::sin(psi);
          x_refs = {};
          y_refs = {};
          for (size_t i=0; i < ptsx.size(); ++i) {
            double x_local = cos_psi*(ptsx[i] - px) + sin_psi*(ptsy[i] - py);
            double y_local = cos_psi*(ptsy[i] - py) - sin_psi*(ptsx[i] - px);
            x_refs.push_back(x_local);
            y_refs.push_back(y_local);
          }
          // fit polynomial to updated reference trajectory coordinates
          xpoints = Eigen::Map<Eigen::VectorXd>(x_refs.data(), x_refs.size());
          ypoints = Eigen::Map<Eigen::VectorXd>(y_refs.data(), y_refs.size());
          coeffs = polyfit(xpoints, ypoints, 2);
          //if delay is true, then we would have to update the states.
          if (delay) {
            x += v * std::cos(psi_local) * dt;
            y += v * std::sin(psi_local) * dt;
            std::vector<double> f_df = polyeval(coeffs, x);
            psi_local -= v/Lf * steer_value * deg2rad(25.0) * dt;
            v += throttle_value * dt;
            cte += v * std::sin(epsi) * dt;
            epsi -= v/Lf * steer_value * deg2rad(25.0) * dt;
          }

          state << x, y, psi_local, v, cte, epsi;

          // std::cout << "State: " << std::endl <<  state << std::endl;

          std::vector<std::vector<double>> result = mpc.Solve(state, coeffs);
          steer_value = result[2][0]/deg2rad(25.0);
          throttle_value = result[2][1];

          // Lets Store the value to visualize the signals provided by the MPC
          steering_data.push_back(steer_value);
          acceleration_data.push_back(throttle_value);
          cte_data.push_back(cte);
          epsi_data.push_back(epsi);
          data_position[0].push_back(ptsx[0]);
          data_position[1].push_back(ptsy[1]);
          data_position[2].push_back(px);
          data_position[3].push_back(py);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(delay));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

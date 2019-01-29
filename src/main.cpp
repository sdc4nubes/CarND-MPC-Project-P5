#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // std::cout << sdata << std::endl;
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
					double delta = j[1]["steering_angle"];
					double a = j[1]["throttle"];

					// Declare Eigen vectors for polyfit
					Eigen::VectorXd ptsx_car(ptsx.size());
					Eigen::VectorXd ptsy_car(ptsy.size());

					// Calculate vehicle's coordinates
					for (int i = 0; i < ptsx.size(); i++) {
						double x = ptsx[i] - px;
						double y = ptsy[i] - py;
						ptsx_car[i] = x * cos(-psi) - y * sin(-psi);
						ptsy_car[i] = x * sin(-psi) + y * cos(-psi);
					}
					// Fit a 3rd-order polynomial to the above coordinates
					auto coeffs = polyfit(ptsx_car, ptsy_car, 3);

					// Calculate the cross track error
					double cte = polyeval(coeffs, 0);

					// Calculate the orientation error
					double epsi = -atan(coeffs[1]);

					// Set the length from front to CoG
					const double Lf = 2.67;

					// Set latency
					const double dt = 0.1;

					// Predict the state after latency
					double pred_px = 0.0 + v * dt;
					const double pred_py = 0.0;
					double pred_psi = 0.0 + v * -delta / Lf * dt;
					double pred_v = v + a * dt;
					double pred_cte = cte + v * sin(epsi) * dt;
					double pred_epsi = epsi + v * -delta / Lf * dt;

					// Set predicted state values
					Eigen::VectorXd state(6);
					state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;

					// Set the MPC
					//auto vars = mpc.Solve(state, coeffs);

					// Calculate steering and throttle
					double steer_value = vars[0] / (deg2rad(25) * Lf);
					double throttle_value = vars[1];

					// Set steering and throttle
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory 
					vector<double> mpc_x_vals = { state[0] };
					vector<double> mpc_y_vals = { state[1] };

					// Add (x,y) points to the list.
					// These points in the simulator are connected by a Green line
					for (int i = 2; i < vars.size(); i += 2) {
						mpc_x_vals.push_back(vars[i]);
						mpc_y_vals.push_back(vars[i + 1]);
					}
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

					// Add (x,y) points to list.
					// These points in the simulator are connected by a Yellow line
					double poly_inc = 2.5;
					int num_points = 25;
					for (int i = 1; i < num_points; i++) {
						next_x_vals.push_back(poly_inc * i);
						next_y_vals.push_back(polyeval(coeffs, poly_inc * i));
					}
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;

          // Latency
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
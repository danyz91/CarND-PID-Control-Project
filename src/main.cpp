#include <math.h>
#include <uWS/uWS.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>

#include "PID.h"
#include "json.hpp"

#define MAX_SPEED 30  // MPH

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double clamp_value(double n, double lower, double upper) {
  return std::max(lower, std::min(n, upper));
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Initialize the pid variable.

  PID steer_pid;

  steer_pid.Init(0.1, 0.0001, 2.0);

  PID speed_pid;

  speed_pid.Init(0.3, 0.0001, 2.0);

  h.onMessage([&steer_pid, &speed_pid](uWS::WebSocket<uWS::SERVER> ws,
                                       char *data, size_t length,
                                       uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          double speed_value;
          // Compute speed error
          double speed_error = speed - MAX_SPEED;

          // Update steering and speed PID errors
          steer_pid.UpdateError(cte);
          speed_pid.UpdateError(speed_error);

          steer_value = steer_pid.TotalError();
          speed_value = speed_pid.TotalError();

          // Clamp steer value to [-1, 1]
          steer_value = clamp_value(steer_value, -1.0, 1.0);
          // Clamp throttle value to [0, 1]
          speed_value = clamp_value(speed_value, 0.0, 1.0);

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = speed_value;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  });  // end h.onMessage

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

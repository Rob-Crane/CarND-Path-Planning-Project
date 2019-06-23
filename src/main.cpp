#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include "utils.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  RouteFrame rf(map_waypoints_s, map_waypoints_x, map_waypoints_y);

  EgoState state;

  h.onMessage([&rf, &state](uWS::WebSocket<uWS::SERVER> ws, char *data,
                            size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          state.inertial_ << j[1]["x"], j[1]["y"];
          state.route_ << j[1]["s"], j[1]["d"];
          state.yaw_ = j[1]["yaw"];
          state.s_ = j[1]["s"];

          // Previous path data given to the Planner
          Path prevPath = {.x_ = j[1]["previous_path_x"],
                               .y_ = j[1]["previous_path_y"]};

          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Set vehicle S to end of previous traj.
          state.s_ = prevPath.size() == 0 ? state.s_ : end_path_s;

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          // Use sensor fusion data to determine which lanes are occupied.
          LaneOccupancy occupancy;
          for (const auto &agent : sensor_fusion) {
            int agent_lane = get_lane(agent[6]);
            Eigen::Vector2d agent_vel;
            agent_vel << agent[3], agent[4];
            double speed = agent_vel.norm();
            double agent_s = (double)agent[5] + prevPath.size()*0.02*speed;

            double egoAheadBound = state.s_ + 25;
            double egoBehindBound = state.s_ - 25;
            if (state.lane_ == agent_lane) {
              occupancy.ahead_ = occupancy.ahead_ ||
                                 (agent_s > state.s_ && (agent_s - state.s_) < 25);
            } else if (state.lane_ > agent_lane) {
              occupancy.left_ = occupancy.left_ ||
                  (agent_s > egoBehindBound && agent_s < egoAheadBound);
            } else if (state.lane_ < agent_lane) {
              occupancy.right_ = occupancy.right_ ||
                  (agent_s > egoBehindBound && agent_s < egoAheadBound);
            }
          }

          Behavior behavior = decide(occupancy, state);
          state.lane_ = behavior.target_lane_;
          Spline spline = computeSpline(state, prevPath, rf);
          Path newPath = prevPath;  // Copy previous points.

          // Get a distant point along spline.
          double targX = 30.0;
          double targY = spline.spline_(targX);
          double dist = sqrt(targX * targX + targY * targY);

          double lastX = 0;

          for (int i = 1; i < 50 - prevPath.size(); i++) {
            state.speed_ += behavior.acc_;
            if (state.speed_ > kMaxSpeed) {
              state.speed_ = kMaxSpeed;
            } else if (state.speed_ < kMaxAccel) {
              state.speed_ = kMaxAccel;
            }

            double stepDist = 0.02 * state.speed_;
            double N = dist / stepDist;
            double x_local = lastX + targX / N;
            double y_local = spline.spline_(x_local);
            lastX = x_local;
            double x_global = x_local * cos(spline.base_yaw_) - y_local * sin(spline.base_yaw_);
            double y_global = x_local * sin(spline.base_yaw_) + y_local * cos(spline.base_yaw_);
            x_global += spline.base_x_;
            y_global += spline.base_y_;
            newPath.x_.push_back(x_global);
            newPath.y_.push_back(y_global);
          }

          json msgJson;
          msgJson["next_x"] = newPath.x_;
          msgJson["next_y"] = newPath.y_;
          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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

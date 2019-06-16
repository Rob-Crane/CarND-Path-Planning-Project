#include <uWS/uWS.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "types.h"
#include "utils.h"

#include "route/route_frame.h"
#include "trajectory/trajectory.h"
#include "trajectory/types.h"

#include <thread>
// for convenience
using nlohmann::json;
using std::string;
using std::vector;

namespace path_planner {
int run() {
  uWS::Hub h;

  BasicMap map = load_map();
  std::shared_ptr<RouteFrame> rf = std::make_shared<RouteFrame>(
      map.map_waypoints_x, map.map_waypoints_y, map.map_waypoints_s);
  std::vector<TrajectoryState> last_trajectory;

  h.onMessage([&rf, &last_trajectory](uWS::WebSocket<uWS::SERVER> ws,
                                      char* data, size_t length,
                                      uWS::OpCode opCode) {
    time_point start_time = steady_clock::now();
    // "42" at the start of the message means there's a websocket message
    // event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          std::cout << "======================" << std::endl;

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          std::cout << "car_x: " << car_x << " car_y: " << car_y
                    << " car_speed: " << car_speed << " car_s: " << car_s;

          TrajectoryPoint curr_pos(InertialCoordinate(car_x, car_y), rf);
          std::cout << " xform_s: " << curr_pos.route().s() << std::endl;
          // Find current position in last trajectory.
          int i = 0;
          while (i < last_trajectory.size() &&
                 last_trajectory[i].s() < curr_pos.route().s()) {
            ++i;
          }
          std::cout << "===Waypoint list recycle.===\n";
          constexpr int kBuffer = 10;
          std::vector<TrajectoryState> new_trajectory;
          int j;
          for (j = std::max(i-1, 0); j < last_trajectory.size() && j < i + kBuffer; ++j) {
            new_trajectory.push_back(last_trajectory[j]);
          }
          if (new_trajectory.empty()) {
            new_trajectory.emplace_back(curr_pos, 0.0, 0.0);
          }
          std::cout << "recycled traj size: " << new_trajectory.size()
                    << std::endl;

          TrajectoryState s0(new_trajectory.back());
          std::cout << "Generating traj from s: " << s0.s()
                    << " sv: " << s0.sv() << " sa: " << s0.sa() <<" t: " << start_time.time_since_epoch().count() << std::endl;
          UnblockedLongitudinalTrajectory traj(s0.s(), s0.sv(), s0.sa(),
                                               start_time);
          // Previous path data given to the Planner
          // auto previous_path_x = j[1]["previous_path_x"];
          // auto previous_path_y = j[1]["previous_path_y"];
          //// Previous path's end s and d values
          // double end_path_s = j[1]["end_path_s"];
          // double end_path_d = j[1]["end_path_d"];

          //// Sensor Fusion Data, a list of all other cars on the same side
          ////   of the road.
          // auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will
           * visit
           *   sequentially every .02 seconds (20ms)
           */
          std::cout << "last_trajectory:" << std::endl;
          std::cout << last_trajectory.size() << std::endl;
          for (int ii = 0; ii < last_trajectory.size(); ++ii) {
            if (ii == i) {
              std::cout << ii << "^ s: ";
            } else if (ii == j) {
              std::cout << ii << "v s: ";
            } else {
              std::cout << ii << "| s: ";
            }
            const auto& p = last_trajectory[ii];
            std::cout << p.s() << " x: " << p.x() << " y: " << p.y()
                      << " sv: " << p.sv() << " sa: " << p.sa() << std::endl;
          }

          for (seconds t = seconds(0.02); t <= seconds(1.0);
               t += seconds(0.02)) {
            KinematicPoint kp(traj.at(t + start_time));
            RouteCoordinate r(kp.x_, s0.d());
            TrajectoryPoint pt(r, rf);
            new_trajectory.emplace_back(pt, kp.v_, kp.a_);
          }

          std::cout << "new_trajectory:" << std::endl;
          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;
          std::vector<double> next_s_vals;
          
          constexpr double kMinSDiff = 0.009;
          last_trajectory.clear();
          for (TrajectoryState& s : new_trajectory) {
            if (!next_s_vals.empty() && s.s() - next_s_vals.back() < kMinSDiff)
              continue;
          next_x_vals.push_back(s.x());
          next_y_vals.push_back(s.y());
          next_s_vals.push_back(s.s());
          last_trajectory.push_back(s);
          std::cout << "s: " << s.s() << " x: " << next_x_vals.back()
                    << " y: " << next_y_vals.back() << " sv: " << s.sv()
                    << " sa: " << s.sa() << std::endl;
        }
        // TODO, add extra kinemetic values to TrajectoryPoint

        msgJson["next_x"] = std::move(next_x_vals);
        msgJson["next_y"] = std::move(next_y_vals);

        auto msg = "42[\"control\"," + msgJson.dump() + "]";

        // if (car_speed > 23.0) {
        // std::cout << "Overspeed exit." << std::endl;
        // std::exit(1);
        //}
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        last_trajectory = std::move(new_trajectory);
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

h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char* message,
                       size_t length) {
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
}  // path_planner

int main() { return path_planner::run(); }

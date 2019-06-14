#include <uWS/uWS.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "utils.h"

#include "route/route_frame.h"
#include "trajectory/trajectory.h"
#include "trajectory/types.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  path_planner::BasicMap map = path_planner::load_map();
  std::shared_ptr<path_planner::RouteFrame> rf =
      std::make_shared<path_planner::RouteFrame>(
          map.map_waypoints_x, map.map_waypoints_y, map.map_waypoints_s);

  h.onMessage([&rf](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                    uWS::OpCode opCode) {
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

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          path_planner::InertialCoordinate ic(car_x, car_y);
          path_planner::TrajectoryPoint tp(ic, rf);
          path_planner::time_point now = path_planner::steady_clock::now();
          path_planner::RouteCoordinate rc(tp.route());
          path_planner::UnblockedLongitudinalTrajectory traj(rc.s(), car_speed,
                                                             0.0, now);

          //// Previous path data given to the Planner
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
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds (20ms)
           */
          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;
          for (path_planner::seconds t = path_planner::seconds(0.02);
               t < path_planner::seconds(2.0);
               t += path_planner::seconds(0.02)) {
            path_planner::RouteCoordinate r(traj.at(t + now), rc.d());
            std::cout<<"s: " << r.s()<<std::endl;
            path_planner::TrajectoryPoint tp(r, rf);
            path_planner::InertialCoordinate i = tp.inertial();
            next_x_vals.push_back(i.x());
            next_y_vals.push_back(i.y());
          }
          std::exit(1);

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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

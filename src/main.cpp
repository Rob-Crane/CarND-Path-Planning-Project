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

#include "decision/decision.h"
#include "decision/types.h"
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
  std::cout<<"Building course route, may take a few seconds..."<<std::endl;
  std::shared_ptr<RouteFrame> routeFrame = std::make_shared<RouteFrame>(
      map.map_waypoints_x, map.map_waypoints_y, map.map_waypoints_s);

  Decision decision(routeFrame);
  h.onMessage([&decision](uWS::WebSocket<uWS::SERVER> ws,
                                            char* data, size_t length,
                                            uWS::OpCode opCode) {
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
         std::cout<<std::chrono::steady_clock::now().time_since_epoch().count()<<"|begin->"<<std::flush;
          // j[1] is the data JSON object

          // Main car's localization Data
          InertialVector egoPos(j[1]["x"], j[1]["y"]);

          auto sensor_fusion = j[1]["sensor_fusion"];
          constexpr size_t kSenseX = 1;
          constexpr size_t kSenseY = 2;
          constexpr size_t kSenseDX = 3;
          constexpr size_t kSenseDY = 4;
          std::vector<AdversaryObservation> adversaries;

          for (auto f : sensor_fusion) {
            adversaries.emplace_back(f[kSenseX], f[kSenseY], f[kSenseDX],
                                     f[kSenseDY]);
          }
          std::vector<TrajectoryState> decision_trajectory = decision.plan(adversaries, egoPos);

          json msgJson;

          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;

          for (TrajectoryState& s : decision_trajectory) {
            next_x_vals.push_back(s.x());
            next_y_vals.push_back(s.y());
          }

          msgJson["next_x"] = std::move(next_x_vals);
          msgJson["next_y"] = std::move(next_y_vals);

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          std::cout<<"sending->"<<std::flush;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          std::cout<<"sent"<<std::endl;
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
                         char* message, size_t length) {
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

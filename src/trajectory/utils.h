#pragma once

#include "trajectory/traj_types.h"

namespace path_planner {

//constexpr Lane[] kLanes = {Lane(0, 0.0, 2.0, 4.0), Lane(1, 4.0, 6.0, 8.0),
                    //Lane(2, 8.0, 10.0, 12.0), Lane(3, 12.0, 14.0, 16.0)};

namespace {
  constexpr double kLaneWidth = 4.0;
  constexpr int  kMaxLane = 4;
}

Dx next_lane_midpoint(Dx x) {
  int current_lane = std::floor(x / kLaneWidth);
  int next_lane = std::min(kMaxLane, current_lane + 1);
  return 2.0 + next_lane * kLaneWidth;
}

Dx prev_lane_midpoint(Dx x) {
  int current_lane = std::floor(x / kLaneWidth);
  int prev_lane = std::max(0, current_lane - 1);
  return 2.0 + prev_lane * kLaneWidth;
}

}  // path_planner

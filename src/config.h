#pragma once
namespace path_planner {
// For continuity, take kBuffer points from the last planned
// trajectory starting immediately becore current position.
constexpr int kBuffer = 10;

const double kAheadFilter = 50.0;
constexpr double kRouteLength = 6945.554;
constexpr double kLaneWidth = 2.001;
constexpr unsigned kMaxLane = 3;         // index of right most lane.
}  // path_planner

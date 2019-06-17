namespace path_planner {
// For continuity, take kBuffer points from the last planned
// trajectory starting immediately becore current position.
constexpr int kBuffer = 10;

const double kAheadFilter = 50.0;
constexpr double kRouteLength = 6945.554;
}  // path_planner

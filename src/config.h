#pragma once
namespace path_planner {
// For continuity, take kBuffer points from the last planned
// trajectory starting immediately becore current position.
constexpr int kBuffer = 7;

const double kAheadFilter = 50.0;
constexpr double kRouteLength = 6947.45;
constexpr double kLaneWidth = 4.000;
constexpr unsigned kMaxLane = 3;         // index of right most lane.
constexpr double kHalfEnvelopeWidth = 2.1;
constexpr double kMaxLaneDist = 100.0;
constexpr double kLaneUpdateWeight = 0.05;
// Small delta-S values in published trajectory cause instablility
// in simulator.  Filter small values.
constexpr double kMinSDiff = 0.011;
constexpr double kMaintainDistance = 6.0;  // Following distance
//constexpr double kVClose = 17.0;         // Est. speed to close to leading veh.
constexpr double kVMax = 19.00;             // Speed limit
constexpr double kVmaxBuffer = 3.0;
constexpr double kDecel = -1.3;       // Est. braking accel
constexpr double kUnblockedAvgAcc = 1.9;        // Est. avg accel
constexpr double kFollowAvgAcc = 1.0;        // Est. avg accel
constexpr double kFollowVelGain = 0.85;  // Want to avoid perpetually closing.
//constexpr double kFollowSafety = 10.0;
constexpr double kMinIntercept = 10.0; // Seconds minimum to intercept.
}  // path_planner

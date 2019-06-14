#pragma once

#include <cmath>

#include "Eigen-3.3/Eigen/Dense"

#include "types.h"

namespace path_planner {

Dx next_lane_midpoint(Dx x, const double kLaneWidth, const int kMaxLane);
Dx prev_lane_midpoint(Dx x, const double kLaneWidth);

Eigen::Matrix3d get_tmat(double dt);

// Get approximate closing time to a target driving in lane.  Estimate by
// assuming simple model of instant acceleration to vclose, maintaining
// vclose, then decelerating to match target speed.  If solution doens't exist,
// use nominal value (target is close to Ego).
KinematicPoint steady_state_follow_estimate(KinematicPoint p0,
                                            KinematicPoint blocking,
                                            double a_acc, double a_dec,
                                            double vclose, double xbuff,
                                            double nominal_intercept);

KinematicPoint steady_state_max_speed_estimate(KinematicPoint p0, double a_acc,
                                               double vmax);

}  // path_planner

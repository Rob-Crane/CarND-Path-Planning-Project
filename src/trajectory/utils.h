#pragma once

#include <cmath>

#include "Eigen-3.3/Eigen/Dense"

#include "types.h"

namespace path_planner {

int lane_number(Dx x);
bool inEnvelope(Dx ego, Dx adv);
Dx next_lane_midpoint(Dx x);
Dx prev_lane_midpoint(Dx x);
double midpoint(int lane_number);

Eigen::Matrix3d get_tmat(double dt);

// Get approximate closing time to a target driving in lane.  Estimate by
// assuming simple model of instant acceleration to vclose, maintaining
// vclose, then decelerating to match target speed.  If solution doens't exist,
// use nominal value (target is close to Ego).
KinematicPoint steady_state_follow_estimate(KinematicPoint p0,
                                            KinematicPoint blocking);

KinematicPoint steady_state_max_speed_estimate(KinematicPoint p0);

}  // path_planner

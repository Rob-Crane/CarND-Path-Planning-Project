#include <cassert>

#include "spline.h"

struct LaneOccupancy {
  bool left_ = false;
  bool right_ = false;
  bool ahead_ = false;
};

struct EgoState {
  int lane_ = 1;
  double speed_ = 0.0;
  Eigen::Vector2d inertial_;  // [x, y]
  Eigen::Vector2d route_;     // [s, d]
  double yaw_ = -1.0;
  double s_ = -1.0;
};

constexpr double kMaxAccel = 0.1;
constexpr double kMaxSpeed = 21.0;

struct Behavior {
  int target_lane_ = 1;
  double acc_ = 0.0;
};

struct Path {
  vector<double> x_;
  vector<double> y_;
  size_t size() const {
    assert(x_.size() == y_.size());
    return x_.size();
  }
};

Behavior decide(const LaneOccupancy& lane_occ, const EgoState& ego) {
  Behavior ret;
  ret.target_lane_ = ego.lane_;  // Default to stay-in-lane.
  if (lane_occ.ahead_) {
    if (ego.lane_ >= 1 && !lane_occ.left_) {
      ret.target_lane_ -= 1;
    } else if (ego.lane_ <= 1 && !lane_occ.right_) {
      ret.target_lane_ += 1;
    } else {
      ret.acc_ = -kMaxAccel;
    }
  } else if (ego.speed_ < kMaxSpeed) {
    ret.acc_ = kMaxAccel;
  }
  return ret;
}

class RouteFrame {
    public:
  RouteFrame(const vector<double>& map_s, const vector<double>& map_x,
             const vector<double>& map_y)
      : map_waypoints_s_(map_s),
        map_waypoints_x_(map_x),
        map_waypoints_y_(map_y) {}

  std::vector<double> getInertial(double s, double d) const {
    return getXY(s, d, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
  }

 private:
  vector<double> map_waypoints_s_;
  vector<double> map_waypoints_x_;
  vector<double> map_waypoints_y_;
};

double midpoint(int lane) { return 2.0 + 4.0 * lane; }
int get_lane(double d) { return std::floor(d / 4.0); }

struct Spline {
  tk::spline spline_;
  double base_yaw_ = -1.0;
  double base_x_ = -1.0;
  double base_y_ = -1.0;
};

Spline computeSpline(const EgoState& ego, const Path& prevPath, const RouteFrame& rf) {
  vector<double> fitX;
  vector<double> fitY;

  Spline spline;

  if (prevPath.size() < 2) {
    spline.base_x_ = ego.inertial_[0];
    spline.base_y_ = ego.inertial_[1];
    spline.base_yaw_ = deg2rad(ego.yaw_);

    fitX.push_back(ego.inertial_[0] - cos(ego.yaw_));
    fitX.push_back(ego.inertial_[0]);

    fitY.push_back(ego.inertial_[1] - sin(ego.yaw_));
    fitY.push_back(ego.inertial_[1]);

  } else {
    spline.base_x_ = prevPath.x_[prevPath.size() - 1];
    spline.base_y_ = prevPath.y_[prevPath.size() - 1];

    double fitX0 = prevPath.x_[prevPath.size() - 2];
    double fitY0 = prevPath.y_[prevPath.size() - 2];
    spline.base_yaw_ = atan2(spline.base_y_ - fitY0, spline.base_x_ - fitX0);

    fitX.push_back(fitX0);
    fitX.push_back(spline.base_x_);

    fitY.push_back(fitY0);
    fitY.push_back(spline.base_y_);
  }

  double d = midpoint(ego.lane_);
  auto wp0 = rf.getInertial(ego.s_+30, d);
  auto wp1 = rf.getInertial(ego.s_+60, d);
  auto wp2 = rf.getInertial(ego.s_+90, d);

  fitX.push_back(wp0[0]);
  fitX.push_back(wp1[0]);
  fitX.push_back(wp2[0]);

  fitY.push_back(wp0[1]);
  fitY.push_back(wp1[1]);
  fitY.push_back(wp2[1]);

  for (int i = 0; i < fitX.size(); i++) {
    double transX = fitX[i] - spline.base_x_;
    double transY = fitY[i] - spline.base_y_;

    fitX[i] = transX * cos(0 - spline.base_yaw_) - transY * sin(0 - spline.base_yaw_);
    fitY[i] = transX * sin(0 - spline.base_yaw_) + transY * cos(0 - spline.base_yaw_);
  }

  spline.spline_.set_points(fitX, fitY);
  return spline;
}


#include <unordered_map>

#include "decision/types.h"
#include "route/route_frame.h"
#include "trajectory/trajectory.h"

namespace path_planner {

class LaneDecider {
  int update(double lane_mins[3], int current) {
    if (current != current_) {
      current_ = current;
      weights_ = {0.0, 0.0, 0.0};
      return current_;
    }

    int lo_lane = std::max(0, current-1);
    int hi_lane = std::min(2, current+1);
    double total = 0.0;
    for(int i = lo_lane; i < hi_lane; ++i) {
        total+=lane_mins[i];
    }
    double invUpdate = 1 - kLaneUpdateWeight;
    for(int i = lo_lane; i < hi_lane; ++i) {
      double normalized = lane_mins[i] / total;
      weights_[i] = invUpdate * weights_[i] + kLaneUpdateWeight * normalized;
    }
    for(int i = lo_lane; i < hi_lane; ++i) {
       if(weights_[i] > 0.5) {
         return i;
       }
    }
    return current_;
  }

 private:
  int weights_[3];
  int current_;
}

class Decision {
 public:
  Decision(std::shared_ptr<RouteFrame> route_frame)
      : route_frame_(std::move(route_frame)) {}
  std::vector<TrajectoryState> plan(
      const std::vector<AdversaryObservation> adversaries,
      InertialVector egoPos);

 private:
  int target_lane_;
  std::vector<TrajectoryState> last_trajectory_;
  std::shared_ptr<RouteFrame> route_frame_;
  // Sim bug causes unreliable inertial coordinates at route end,
  // just use dead reckoning after that point.
  double last_x_ = -1.0;
  LaneDecider lane_decider_;
};

}  // path_planner

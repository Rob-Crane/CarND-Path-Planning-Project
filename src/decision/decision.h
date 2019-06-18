#include <unordered_map>

#include "decision/types.h"
#include "route/route_frame.h"
#include "trajectory/trajectory.h"

namespace path_planner {

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
};

}  // path_planner

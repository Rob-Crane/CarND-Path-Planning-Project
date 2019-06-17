
namespace path_planner {

struct AdversaryObservation {
  AdversaryObservation(double x, double y, double dx, double dy)
      : x_(x), y_(y), dx_(dx), dy_(dy) {}
  double x_;
  double y_;
  double dx_;
  double dy_;
};
}  // path_planner

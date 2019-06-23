#Path Generation

The path planner relies on four components:
* Lane Occupancy detection.
* Behavior Decision
* Smooth Path Generator
* Simple Vehicle Controller

## Lane Occupancy Detection
The lane occupancy detection provides 3 booleans as output which represent whether the physical lanes to the left or right, or the current lane ahead are occupied.  Other vehicle S-values along the road are considered out to a maximum distance.

## Behavior Decision
With the results of lane occupancy detection, the behavior decision follows the following rules:
If the lane ahead is blocked,
* Change lanes to left or right if possible.
* Slow down.
If the lane ahead is no blocked,
* Speed up to maximum velocity.

# Smooth Path Generator
The smooth path generator uses recent vehicle path points and map waypoints to fit a cubic spline.  The cubic spline is used to generate a smooth lateral driving profile.

## Simple Vehicle Controller
The simple vehicle controller uses a basic model of fixed acceleration/decceleration that adjusts the ego vehicle velocity at each vehicle tick.  The controller creates points along the spline path recycling the points from the previous tick.

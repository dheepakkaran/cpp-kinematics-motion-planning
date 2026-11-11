// Joint-space motion planning (OMPL RRT-Connect) + trajectory optimization
// (OMPL shortcutting / vertex reduction) + time parameterization under velocity and
// acceleration limits. Collision model: spheres along the links vs axis-aligned boxes
// (no self-collision check; see README limits).
#pragma once

#include <vector>

#include "kinmp/kinematics.hpp"

namespace kinmp {

struct Box {
  Eigen::Vector3d lo, hi;
};

struct PlanResult {
  bool ok = false;
  double plan_time_s = 0;      // RRT-Connect solve time
  double optimize_time_s = 0;  // shortcut + vertex reduction time
  double len_raw = 0, len_opt = 0;  // joint-space path length (rad, L2)
  int n_raw = 0, n_opt = 0;         // waypoints
  bool opt_valid = false;           // optimized path re-checked state by state
  std::vector<Vec> path;            // optimized waypoints
  std::vector<Vec> raw_path;        // RRT-Connect waypoints before optimization
};

// Rest-to-rest trapezoidal profile per segment, all joints synchronized to the slowest one.
class Trajectory {
 public:
  Trajectory() = default;
  Trajectory(const std::vector<Vec>& wps, double vmax, double amax);
  double duration() const { return t_.empty() ? 0 : t_.back(); }
  Vec sample(double t) const;
  bool empty() const { return wps_.empty(); }
  double peak_speed() const { return peak_v_; }

 private:
  std::vector<Vec> wps_;
  std::vector<double> t_;   // cumulative time at each waypoint
  std::vector<double> T_;   // segment duration
  std::vector<double> ta_;  // segment accel time
  double peak_v_ = 0;
};

class Planner {
 public:
  Planner(Chain chain, std::vector<Box> boxes, double sphere_radius = 0.06);
  bool valid(const Vec& q) const;
  PlanResult plan(const Vec& start, const Vec& goal, double timeout_s = 2.0, unsigned seed = 1) const;
  const Chain& chain() const { return chain_; }
  const std::vector<Box>& boxes() const { return boxes_; }
  double radius() const { return r_; }

 private:
  Chain chain_;
  std::vector<Box> boxes_;
  double r_;
};

}  // namespace kinmp

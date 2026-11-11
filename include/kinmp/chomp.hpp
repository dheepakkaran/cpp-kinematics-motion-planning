// CHOMP-style trajectory optimization in joint space (Ratliff et al. 2009, simplified):
// N waypoints, endpoints fixed, cost = w_smooth * 0.5 * sum ||q[i-1] - 2 q[i] + q[i+1]||^2
// + w_obs * sum over waypoints and link spheres of c(d), d = signed sphere-to-box distance minus radius,
// c(d) = -d + eps/2 (d < 0), (d - eps)^2 / (2 eps) (0 <= d <= eps), 0 otherwise.
// Update: covariant gradient step  xi <- xi - eta * A^-1 grad  (A = smoothness metric K^T K),
// clamped to joint limits, with step halving if the cost goes up.
// Simplification vs the paper: obstacle cost is not weighted by workspace speed and has no
// curvature projection term.
#pragma once

#include "kinmp/planner.hpp"

namespace kinmp {

struct ChompOptions {
  int n_waypoints = 50;
  int max_iters = 300;
  double eta = 0.1;          // step size (fraction of the preconditioned gradient)
  double w_smooth = 1.0;
  double w_obs = 10.0;
  double eps = 0.05;         // clearance margin, m
  double rel_tol = 1e-6;     // stop when relative cost change < rel_tol
};

struct ChompStats {
  double cost_before = 0, cost_after = 0;
  double smooth_before = 0, smooth_after = 0;
  double obs_before = 0, obs_after = 0;
  int iters = 0;
  double time_s = 0;
};

// Resample a waypoint path to n points evenly spaced by joint-space arc length.
std::vector<Vec> resample(const std::vector<Vec>& path, int n);
double path_length(const std::vector<Vec>& path);
double smoothness_cost(const std::vector<Vec>& path);  // 0.5 * sum ||second difference||^2
// Minimum signed clearance (sphere surface to nearest box, m) over the path checked every 0.01 rad.
double min_clearance(const Planner& pl, const std::vector<Vec>& path);
bool dense_valid(const Planner& pl, const std::vector<Vec>& path, double step = 0.01);

std::vector<Vec> chomp(const Planner& pl, const std::vector<Vec>& init, const ChompOptions& o, ChompStats* st);

}  // namespace kinmp

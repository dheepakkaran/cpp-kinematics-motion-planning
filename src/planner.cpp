#include "kinmp/planner.hpp"

#include <algorithm>
#include <chrono>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace kinmp {

Trajectory::Trajectory(const std::vector<Vec>& wps, double vmax, double amax) : wps_(wps) {
  t_.push_back(0);
  for (size_t i = 1; i < wps_.size(); ++i) {
    const double d = (wps_[i] - wps_[i - 1]).cwiseAbs().maxCoeff();  // slowest joint distance
    double T, ta;
    if (d >= vmax * vmax / amax) {
      ta = vmax / amax;
      T = d / vmax + ta;
    } else {
      ta = std::sqrt(d / amax);
      T = 2 * ta;
    }
    if (d == 0) T = ta = 0;
    T_.push_back(T);
    ta_.push_back(ta);
    t_.push_back(t_.back() + T);
    if (T > 0) peak_v_ = std::max(peak_v_, d / (T - ta));
  }
}

Vec Trajectory::sample(double t) const {
  if (wps_.empty()) return Vec();
  if (t <= 0) return wps_.front();
  if (t >= duration()) return wps_.back();
  size_t i = std::upper_bound(t_.begin(), t_.end(), t) - t_.begin();  // segment i-1 .. i
  const size_t s = i - 1;
  const double T = T_[s], ta = ta_[s], u = t - t_[s];
  if (T <= 0) return wps_[i];
  // normalized trapezoid: peak normalized speed vp so that area = 1
  const double vp = 1.0 / (T - ta);
  double x;
  if (u < ta)
    x = 0.5 * vp / ta * u * u;
  else if (u < T - ta)
    x = 0.5 * vp * ta + vp * (u - ta);
  else {
    const double w = T - u;
    x = 1.0 - 0.5 * vp / ta * w * w;
  }
  return wps_[s] + std::clamp(x, 0.0, 1.0) * (wps_[i] - wps_[s]);
}

namespace {
double sphere_box_dist(const Eigen::Vector3d& c, const Box& b) {
  const Eigen::Vector3d p = c.cwiseMax(b.lo).cwiseMin(b.hi);
  return (c - p).norm();
}
}  // namespace

Planner::Planner(Chain chain, std::vector<Box> boxes, double sphere_radius)
    : chain_(std::move(chain)), boxes_(std::move(boxes)), r_(sphere_radius) {}

bool Planner::valid(const Vec& q) const {
  if (!chain_.within_limits(q)) return false;
  const auto F = chain_.frames(q);
  for (size_t i = 1; i < F.size(); ++i) {
    const Eigen::Vector3d a = F[i - 1].translation(), b = F[i].translation();
    const int n = std::max(1, static_cast<int>(std::ceil((b - a).norm() / 0.05)));
    for (int k = 0; k <= n; ++k) {
      const Eigen::Vector3d c = a + (b - a) * (static_cast<double>(k) / n);
      if (c.norm() < 0.25) continue;  // the base itself sits on the floor box
      for (const auto& bx : boxes_)
        if (sphere_box_dist(c, bx) < r_) return false;
    }
  }
  return true;
}

PlanResult Planner::plan(const Vec& start, const Vec& goal, double timeout_s, unsigned seed) const {
  ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
  ompl::RNG::setSeed(seed);
  const int n = chain_.dof();
  auto space = std::make_shared<ob::RealVectorStateSpace>(n);
  ob::RealVectorBounds bounds(n);
  for (int i = 0; i < n; ++i) {
    bounds.setLow(i, chain_.lower()(i));
    bounds.setHigh(i, chain_.upper()(i));
  }
  space->setBounds(bounds);
  og::SimpleSetup ss(space);
  auto to_vec = [n](const ob::State* s) {
    const auto* v = s->as<ob::RealVectorStateSpace::StateType>()->values;
    Vec q(n);
    for (int i = 0; i < n; ++i) q(i) = v[i];
    return q;
  };
  ss.setStateValidityChecker([this, to_vec](const ob::State* s) { return valid(to_vec(s)); });
  ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);  // fraction of space extent
  ob::ScopedState<> a(space), g(space);
  for (int i = 0; i < n; ++i) {
    a[i] = start(i);
    g[i] = goal(i);
  }
  ss.setStartAndGoalStates(a, g);
  ss.setPlanner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));
  PlanResult r;
  const auto t0 = std::chrono::steady_clock::now();
  const auto status = ss.solve(timeout_s);
  const auto t1 = std::chrono::steady_clock::now();
  r.plan_time_s = std::chrono::duration<double>(t1 - t0).count();
  if (status != ob::PlannerStatus::EXACT_SOLUTION) return r;
  og::PathGeometric path = ss.getSolutionPath();
  r.len_raw = path.length();
  for (size_t i = 0; i < path.getStateCount(); ++i) r.raw_path.push_back(to_vec(path.getState(i)));
  r.n_raw = static_cast<int>(path.getStateCount());
  const auto t2 = std::chrono::steady_clock::now();
  og::PathSimplifier simp(ss.getSpaceInformation());
  simp.reduceVertices(path);
  simp.partialShortcutPath(path);  // OMPL 2.0 name for per-joint shortcutting
  simp.reduceVertices(path);
  const auto t3 = std::chrono::steady_clock::now();
  r.optimize_time_s = std::chrono::duration<double>(t3 - t2).count();
  r.len_opt = path.length();
  r.n_opt = static_cast<int>(path.getStateCount());
  r.opt_valid = path.check();
  // dense re-check of the optimized path at 0.01 rad steps
  for (size_t i = 0; i < path.getStateCount(); ++i) r.path.push_back(to_vec(path.getState(i)));
  for (size_t i = 1; i < r.path.size() && r.opt_valid; ++i) {
    const int k = std::max(1, static_cast<int>(std::ceil((r.path[i] - r.path[i - 1]).norm() / 0.01)));
    for (int j = 0; j <= k; ++j)
      if (!valid(r.path[i - 1] + (r.path[i] - r.path[i - 1]) * (static_cast<double>(j) / k))) {
        r.opt_valid = false;
        break;
      }
  }
  r.ok = r.opt_valid;
  return r;
}

}  // namespace kinmp

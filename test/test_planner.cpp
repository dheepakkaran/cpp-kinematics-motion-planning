// Planning tests: 20 Panda queries whose straight joint-space line is blocked by the scene.
// RRT-Connect -> shortcut/vertex reduction -> trapezoidal time parameterization.
// Writes results/planning.json.
#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "kinmp/scene.hpp"

using namespace kinmp;

namespace {
double median(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  return v.empty() ? 0 : (v.size() % 2 ? v[v.size() / 2] : 0.5 * (v[v.size() / 2 - 1] + v[v.size() / 2]));
}
bool line_valid(const Planner& p, const Vec& a, const Vec& b) {
  const int k = std::max(1, static_cast<int>(std::ceil((b - a).norm() / 0.01)));
  for (int j = 0; j <= k; ++j)
    if (!p.valid(a + (b - a) * (static_cast<double>(j) / k))) return false;
  return true;
}
}  // namespace

TEST(Trajectory, RespectsLimitsAndEndpoints) {
  std::vector<Vec> w = {Vec::Zero(3), Vec::Constant(3, 1.0), Vec::Constant(3, 1.05)};
  Trajectory tr(w, 1.0, 2.0);
  EXPECT_LE(tr.peak_speed(), 1.0 + 1e-9);
  EXPECT_TRUE(tr.sample(0).isApprox(w[0]));
  EXPECT_TRUE(tr.sample(tr.duration()).isApprox(w[2]));
  // numerical velocity / acceleration check
  const double dt = 1e-3;
  double vmax = 0, amax = 0;
  for (double t = dt; t < tr.duration() - dt; t += dt) {
    const Vec v = (tr.sample(t + dt) - tr.sample(t - dt)) / (2 * dt);
    const Vec a = (tr.sample(t + dt) - 2 * tr.sample(t) + tr.sample(t - dt)) / (dt * dt);
    vmax = std::max(vmax, v.cwiseAbs().maxCoeff());
    amax = std::max(amax, a.cwiseAbs().maxCoeff());
  }
  EXPECT_LE(vmax, 1.0 + 1e-3);
  EXPECT_LE(amax, 2.0 + 0.05);
}

TEST(Planner, BlockedQueriesPlannedAndShortened) {
  const Planner pl(panda(), demo_scene());
  ASSERT_TRUE(pl.valid(panda_home()));
  std::mt19937 rng(5);
  const double vmax = 1.0, amax = 2.0;
  std::vector<double> t_plan, t_opt, len_raw, len_opt, n_raw, n_opt, dur;
  int queries = 0, ok = 0;
  while (queries < 20) {
    const Vec a = pl.chain().random_q(rng), b = pl.chain().random_q(rng);
    if (!pl.valid(a) || !pl.valid(b) || line_valid(pl, a, b)) continue;  // only blocked queries
    ++queries;
    const PlanResult r = pl.plan(a, b, 5.0, 100 + queries);
    if (!r.ok) continue;
    ++ok;
    EXPECT_TRUE(r.path.front().isApprox(a, 1e-9));
    EXPECT_TRUE(r.path.back().isApprox(b, 1e-9));
    EXPECT_LE(r.len_opt, r.len_raw + 1e-9);
    t_plan.push_back(r.plan_time_s * 1e3);
    t_opt.push_back(r.optimize_time_s * 1e3);
    len_raw.push_back(r.len_raw);
    len_opt.push_back(r.len_opt);
    n_raw.push_back(r.n_raw);
    n_opt.push_back(r.n_opt);
    const Trajectory tr(r.path, vmax, amax);
    dur.push_back(tr.duration());
    EXPECT_LE(tr.peak_speed(), vmax + 1e-9);
  }
  double sum_raw = 0, sum_opt = 0;
  for (size_t i = 0; i < len_raw.size(); ++i) {
    sum_raw += len_raw[i];
    sum_opt += len_opt[i];
  }
  std::ofstream f(std::string(RESULTS_DIR) + "/planning.json");
  f << std::setprecision(6);
  f << "{\n  \"planner\": \"OMPL RRTConnect, joint space (7-DOF Panda), timeout 5 s\",\n"
    << "  \"optimizer\": \"OMPL PathSimplifier reduceVertices + partialShortcutPath + reduceVertices, then trapezoidal time parameterization\",\n"
    << "  \"collision_model\": \"spheres r=0.06 m every 0.05 m along links vs 3 boxes; no self-collision\",\n"
    << "  \"queries_blocked_straight_line\": " << queries << ",\n  \"solved_and_valid\": " << ok << ",\n"
    << "  \"plan_time_ms_median\": " << median(t_plan) << ",\n"
    << "  \"plan_time_ms_max\": " << (t_plan.empty() ? 0 : *std::max_element(t_plan.begin(), t_plan.end())) << ",\n"
    << "  \"optimize_time_ms_median\": " << median(t_opt) << ",\n"
    << "  \"path_len_rad_raw_median\": " << median(len_raw) << ",\n"
    << "  \"path_len_rad_opt_median\": " << median(len_opt) << ",\n"
    << "  \"path_len_rad_raw_total\": " << sum_raw << ",\n"
    << "  \"path_len_rad_opt_total\": " << sum_opt << ",\n"
    << "  \"waypoints_raw_median\": " << median(n_raw) << ",\n"
    << "  \"waypoints_opt_median\": " << median(n_opt) << ",\n"
    << "  \"traj_duration_s_median\": " << median(dur) << ",\n"
    << "  \"vmax_rad_s\": " << vmax << ",\n  \"amax_rad_s2\": " << amax << "\n}\n";
  EXPECT_EQ(ok, queries);
}

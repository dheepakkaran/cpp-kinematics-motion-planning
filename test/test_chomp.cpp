// CHOMP-style optimizer tests + benchmark on the same 20 blocked Panda queries as test_planner.
// Writes results/trajopt.json.
#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <map>

#include "kinmp/chomp.hpp"
#include "kinmp/scene.hpp"

using namespace kinmp;

namespace {
double median(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  return v.empty() ? 0 : (v.size() % 2 ? v[v.size() / 2] : 0.5 * (v[v.size() / 2 - 1] + v[v.size() / 2]));
}
double vmin(const std::vector<double>& v) { return v.empty() ? 0 : *std::min_element(v.begin(), v.end()); }
}  // namespace

TEST(Chomp, SimpleCaseCostDropsEndpointsFixedCollisionFree) {
  // one box far from the arm; jagged initial path around a valid straight line
  const Planner pl(panda(), {{{1.5, -0.1, 0.0}, {1.7, 0.1, 0.5}}});
  const Vec a = panda_home();
  Vec b = a;
  b(0) += 1.0;
  b(3) += 0.5;
  std::vector<Vec> init;
  for (int i = 0; i < 20; ++i) {
    Vec q = a + (b - a) * (i / 19.0);
    if (i > 0 && i < 19) q += 0.15 * std::sin(i * 1.7) * Vec::Ones(7);
    init.push_back(pl.chain().clamp(q));
  }
  ASSERT_TRUE(dense_valid(pl, init));
  ChompStats st;
  const auto out = chomp(pl, init, ChompOptions{}, &st);
  EXPECT_LT(st.cost_after, st.cost_before);
  EXPECT_LT(smoothness_cost(out), smoothness_cost(resample(init, 50)));
  EXPECT_TRUE(out.front() == a);
  EXPECT_TRUE(out.back() == b);
  for (const auto& q : out) EXPECT_TRUE(pl.chain().within_limits(q));
  EXPECT_TRUE(dense_valid(pl, out));
}

TEST(Chomp, BenchmarkOnTwentyBlockedQueries) {
  const Planner pl(panda(), demo_scene());
  std::mt19937 rng(5);  // same query generator as test_planner.cpp
  auto line_valid = [&](const Vec& a, const Vec& b) { return dense_valid(pl, {a, b}); };
  struct M {
    std::vector<double> smooth, len, clear, ms;
    int valid = 0;
  };
  std::map<std::string, M> R;
  int queries = 0, solved = 0, cost_dropped_raw = 0, cost_dropped_sc = 0;
  ChompOptions o;
  while (queries < 20) {
    const Vec a = pl.chain().random_q(rng), b = pl.chain().random_q(rng);
    if (!pl.valid(a) || !pl.valid(b) || line_valid(a, b)) continue;
    ++queries;
    const PlanResult r = pl.plan(a, b, 5.0, 100 + queries);
    if (!r.ok) continue;
    ++solved;
    auto add = [&](const std::string& k, const std::vector<Vec>& p, double ms) {
      auto& m = R[k];
      m.smooth.push_back(smoothness_cost(resample(p, o.n_waypoints)));  // same N for every variant
      m.len.push_back(path_length(p));
      m.clear.push_back(min_clearance(pl, p));
      m.ms.push_back(ms);
      m.valid += dense_valid(pl, p) ? 1 : 0;
    };
    add("rrtconnect_raw", r.raw_path, 0);
    add("shortcut", r.path, r.optimize_time_s * 1e3);
    ChompStats s1, s2;
    const auto c1 = chomp(pl, r.raw_path, o, &s1);
    add("chomp_from_raw", c1, s1.time_s * 1e3);
    cost_dropped_raw += s1.cost_after < s1.cost_before;
    EXPECT_TRUE(c1.front().isApprox(a, 1e-12) && c1.back().isApprox(b, 1e-12));
    const auto c2 = chomp(pl, r.path, o, &s2);
    add("shortcut_then_chomp", c2, (r.optimize_time_s + s2.time_s) * 1e3);
    cost_dropped_sc += s2.cost_after < s2.cost_before;
    EXPECT_TRUE(c2.front().isApprox(a, 1e-12) && c2.back().isApprox(b, 1e-12));
  }
  std::ofstream f(std::string(RESULTS_DIR) + "/trajopt.json");
  f << std::setprecision(6) << "{\n  \"optimizer\": \"CHOMP-style covariant gradient descent, N=" << o.n_waypoints
    << ", eta=" << o.eta << ", w_smooth=" << o.w_smooth << ", w_obs=" << o.w_obs << ", eps=" << o.eps
    << " m, max_iters=" << o.max_iters << "\",\n"
    << "  \"smoothness_cost\": \"0.5 * sum ||q[i-1]-2q[i]+q[i+1]||^2 after resampling each path to N waypoints (rad^2)\",\n"
    << "  \"clearance\": \"min signed distance sphere surface -> box over the path every 0.01 rad (m); negative = penetration\",\n"
    << "  \"queries\": " << queries << ",\n  \"rrt_solved\": " << solved << ",\n"
    << "  \"chomp_cost_decreased_from_raw\": " << cost_dropped_raw << ",\n"
    << "  \"chomp_cost_decreased_from_shortcut\": " << cost_dropped_sc << ",\n  \"variants\": {\n";
  size_t i = 0;
  for (auto& [k, m] : R) {
    f << "    \"" << k << "\": {\"dense_valid\": " << m.valid << ", \"smoothness_median\": " << median(m.smooth)
      << ", \"length_rad_median\": " << median(m.len) << ", \"min_clearance_m_median\": " << median(m.clear)
      << ", \"min_clearance_m_worst\": " << vmin(m.clear) << ", \"time_ms_median\": " << median(m.ms) << "}"
      << (++i < R.size() ? ",\n" : "\n");
  }
  f << "  }\n}\n";
  EXPECT_EQ(cost_dropped_raw, solved);
}

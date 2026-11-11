// Kinematics tests: FK + Jacobian vs MuJoCo (Panda), Jacobian vs central finite differences
// (both chains), palletizer tool-level property, IK success rate + solve time (both chains).
// Writes results/kinematics.json and results/fk_samples.csv (input for matlab/check_kinematics.m).
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>

#include "kinmp/kinematics.hpp"

using namespace kinmp;

namespace {
std::map<std::string, std::string> g_json;  // key -> json value, written at the end

double median(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  return v.empty() ? 0 : (v.size() % 2 ? v[v.size() / 2] : 0.5 * (v[v.size() / 2 - 1] + v[v.size() / 2]));
}
double pct(std::vector<double> v, double p) {
  std::sort(v.begin(), v.end());
  return v.empty() ? 0 : v[std::min(v.size() - 1, static_cast<size_t>(p * v.size()))];
}

Jac fd_jacobian(const Chain& c, const Vec& q, double h = 1e-6) {
  Jac J(6, c.dof());
  for (int i = 0; i < c.dof(); ++i) {
    Vec qp = q, qm = q;
    qp(i) += h;
    qm(i) -= h;
    const auto Tp = c.fk(qp), Tm = c.fk(qm);
    J.block<3, 1>(0, i) = (Tp.translation() - Tm.translation()) / (2 * h);
    const Eigen::AngleAxisd aa(Tp.linear() * Tm.linear().transpose());
    J.block<3, 1>(3, i) = aa.angle() * aa.axis() / (2 * h);
  }
  return J;
}

struct Env : ::testing::Environment {
  void TearDown() override {
    std::ofstream f(std::string(RESULTS_DIR) + "/kinematics.json");
    f << "{\n";
    size_t i = 0;
    for (auto& [k, v] : g_json) f << "  \"" << k << "\": " << v << (++i < g_json.size() ? ",\n" : "\n");
    f << "}\n";
  }
};
const auto* g_env = ::testing::AddGlobalTestEnvironment(new Env);

std::string num(double x) {
  std::ostringstream s;
  s << std::setprecision(6) << x;
  return s.str();
}
}  // namespace

TEST(PandaVsMuJoCo, FkAndJacobianMatchOn1000RandomConfigs) {
  char err[1000] = {0};
  mjModel* m = mj_loadXML(MODEL_PATH, nullptr, err, sizeof(err));
  ASSERT_NE(m, nullptr) << err;
  ASSERT_EQ(m->nq, 7);
  mjData* d = mj_makeData(m);
  const int site = mj_name2id(m, mjOBJ_SITE, "attachment_site");
  ASSERT_GE(site, 0);
  const Chain c = panda();
  std::mt19937 rng(42);
  double max_pos = 0, max_rot = 0, max_jac = 0;
  std::vector<double> jacp(3 * 7), jacr(3 * 7);
  for (int s = 0; s < 1000; ++s) {
    const Vec q = c.random_q(rng);
    for (int i = 0; i < 7; ++i) d->qpos[i] = q(i);
    mj_kinematics(m, d);
    mj_comPos(m, d);
    const auto T = c.fk(q);
    Eigen::Vector3d p(d->site_xpos[3 * site], d->site_xpos[3 * site + 1], d->site_xpos[3 * site + 2]);
    Eigen::Matrix3d R;
    for (int r = 0; r < 3; ++r)
      for (int k = 0; k < 3; ++k) R(r, k) = d->site_xmat[9 * site + 3 * r + k];
    max_pos = std::max(max_pos, (T.translation() - p).norm());
    max_rot = std::max(max_rot, Eigen::AngleAxisd(T.linear() * R.transpose()).angle());
    mj_jacSite(m, d, jacp.data(), jacr.data(), site);
    const Jac J = c.jacobian(q);
    for (int r = 0; r < 3; ++r)
      for (int k = 0; k < 7; ++k) {
        max_jac = std::max(max_jac, std::abs(J(r, k) - jacp[7 * r + k]));
        max_jac = std::max(max_jac, std::abs(J(3 + r, k) - jacr[7 * r + k]));
      }
  }
  mj_deleteData(d);
  mj_deleteModel(m);
  g_json["panda_vs_mujoco"] = "{\"samples\": 1000, \"fk_max_pos_err_m\": " + num(max_pos) +
                              ", \"fk_max_rot_err_rad\": " + num(max_rot) +
                              ", \"jacobian_max_abs_err\": " + num(max_jac) + "}";
  EXPECT_LT(max_pos, 1e-9);
  EXPECT_LT(max_rot, 1e-6);
  EXPECT_LT(max_jac, 1e-9);
}

TEST(Jacobian, MatchesFiniteDifferencesBothChains) {
  for (const Chain& c : {panda(), palletizer()}) {
    std::mt19937 rng(7);
    double max_err = 0;
    for (int s = 0; s < 1000; ++s) {
      const Vec q = c.random_q(rng);
      max_err = std::max(max_err, (c.jacobian(q) - fd_jacobian(c, q)).cwiseAbs().maxCoeff());
    }
    g_json["jacobian_vs_fd_" + c.name()] = "{\"samples\": 1000, \"h\": 1e-6, \"max_abs_err\": " + num(max_err) + "}";
    EXPECT_LT(max_err, 1e-6) << c.name();
  }
}

TEST(Palletizer, ToolStaysLevel) {
  const Chain c = palletizer();
  std::mt19937 rng(3);
  double max_tilt = 0;
  for (int s = 0; s < 1000; ++s) {
    const auto T = c.fk(c.random_q(rng));
    const double cosang = std::clamp(-T.linear().col(2).z(), -1.0, 1.0);  // tool z vs world -z
    max_tilt = std::max(max_tilt, std::acos(cosang));
  }
  g_json["palletizer_tool_tilt_max_rad"] = num(max_tilt);
  EXPECT_LT(max_tilt, 1e-6);
}

TEST(IK, SuccessRateAndSolveTimeBothChains) {
  for (const Chain& c : {panda(), palletizer()}) {
    IKOptions o;
    if (c.name() == "palletizer") o.mask = palletizer_mask();
    std::mt19937 rng(11);
    int ok = 0, ok_first = 0;
    std::vector<double> ms, pos_err;
    for (int s = 0; s < 1000; ++s) {
      const Vec q_true = c.random_q(rng);
      const auto target = c.fk(q_true);     // reachable by construction
      const Vec seed = c.random_q(rng);     // independent random seed
      o.seed = 1000 + s;
      const auto t0 = std::chrono::steady_clock::now();
      const IKResult r = solve_ik(c, target, seed, o);
      ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
      if (r.ok) {
        ++ok;
        if (r.attempts == 1) ++ok_first;
        ASSERT_TRUE(c.within_limits(r.q));
        pos_err.push_back((c.fk(r.q).translation() - target.translation()).norm());
      }
    }
    g_json["ik_" + c.name()] = "{\"targets\": 1000, \"success\": " + std::to_string(ok) +
                               ", \"success_first_seed\": " + std::to_string(ok_first) +
                               ", \"max_attempts\": " + std::to_string(o.max_attempts) +
                               ", \"median_ms\": " + num(median(ms)) + ", \"p95_ms\": " + num(pct(ms, 0.95)) +
                               ", \"max_pos_err_m_of_successes\": " +
                               num(pos_err.empty() ? 0 : *std::max_element(pos_err.begin(), pos_err.end())) +
                               ", \"tol_pos_m\": 1e-4, \"tol_rot_rad\": 1e-3}";
    EXPECT_GE(ok, 950) << c.name();
  }
}

TEST(Export, FkSamplesCsvForMatlab) {
  std::ofstream f(std::string(RESULTS_DIR) + "/fk_samples.csv");
  f << "chain";  // 1 = panda, 2 = palletizer
  for (int i = 1; i <= 7; ++i) f << ",q" << i;
  f << ",px,py,pz";
  for (int r = 1; r <= 3; ++r)
    for (int k = 1; k <= 3; ++k) f << ",R" << r << k;
  for (int k = 1; k <= 7; ++k)
    for (int r = 1; r <= 6; ++r) f << ",J" << r << "_" << k;
  f << "\n" << std::setprecision(17);
  int id = 0;
  for (const Chain& c : {panda(), palletizer()}) {
    ++id;
    std::mt19937 rng(100 + id);
    for (int s = 0; s < 100; ++s) {
      const Vec q = c.random_q(rng);
      const auto T = c.fk(q);
      const Jac J = c.jacobian(q);
      f << id;
      for (int i = 0; i < 7; ++i) f << "," << (i < c.dof() ? q(i) : 0.0);
      for (int i = 0; i < 3; ++i) f << "," << T.translation()(i);
      for (int r = 0; r < 3; ++r)
        for (int k = 0; k < 3; ++k) f << "," << T.linear()(r, k);
      for (int k = 0; k < 7; ++k)
        for (int r = 0; r < 6; ++r) f << "," << (k < c.dof() ? J(r, k) : 0.0);
      f << "\n";
    }
  }
  SUCCEED();
}

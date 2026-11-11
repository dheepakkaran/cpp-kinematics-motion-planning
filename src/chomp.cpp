#include "kinmp/chomp.hpp"

#include <chrono>
#include <limits>

namespace kinmp {

namespace {
// Signed distance from point c to box b (negative inside) and its gradient.
double box_sdf(const Eigen::Vector3d& c, const Box& b, Eigen::Vector3d* g) {
  const Eigen::Vector3d ctr = 0.5 * (b.lo + b.hi), half = 0.5 * (b.hi - b.lo);
  const Eigen::Vector3d p = c - ctr;
  const Eigen::Vector3d q = p.cwiseAbs() - half;
  if ((q.array() > 0).any()) {
    const Eigen::Vector3d qp = q.cwiseMax(0.0);
    const double d = qp.norm();
    if (g) *g = (qp.array() * p.array().sign()).matrix() / d;
    return d;
  }
  int k;
  const double d = q.maxCoeff(&k);  // inside: distance to the nearest face (negative)
  if (g) {
    g->setZero();
    (*g)(k) = p(k) >= 0 ? 1.0 : -1.0;
  }
  return d;
}

struct Pt {
  int frame;  // segment from frame-1 to frame
  double s;
};

std::vector<Pt> body_points(const Chain& c, const Vec& q) {
  const auto F = c.frames(q);
  std::vector<Pt> pts;
  for (size_t i = 1; i < F.size(); ++i) {
    const double len = (F[i].translation() - F[i - 1].translation()).norm();
    const int n = std::max(1, static_cast<int>(std::ceil(len / 0.05)));
    for (int k = 0; k <= n; ++k) pts.push_back({static_cast<int>(i), static_cast<double>(k) / n});
  }
  return pts;
}

double cfun(double d, double eps, double* dc) {
  if (d < 0) {
    *dc = -1;
    return -d + 0.5 * eps;
  }
  if (d <= eps) {
    *dc = (d - eps) / eps;
    return (d - eps) * (d - eps) / (2 * eps);
  }
  *dc = 0;
  return 0;
}

// Obstacle cost of one configuration and (optionally) its gradient w.r.t. q.
double obs_cost(const Planner& pl, const Vec& q, double eps, Vec* grad) {
  const Chain& c = pl.chain();
  const auto F = c.frames(q);
  std::vector<Eigen::Matrix<double, 3, Eigen::Dynamic>> JO;
  if (grad) {
    JO = c.origin_jacobians(q);
    grad->setZero(c.dof());
  }
  double cost = 0;
  for (const Pt& p : body_points(c, q)) {
    const Eigen::Vector3d x = (1 - p.s) * F[p.frame - 1].translation() + p.s * F[p.frame].translation();
    if (x.norm() < 0.25) continue;  // same base exclusion as Planner::valid
    double dmin = std::numeric_limits<double>::infinity();
    Eigen::Vector3d gmin = Eigen::Vector3d::Zero();
    for (const Box& b : pl.boxes()) {
      Eigen::Vector3d g;
      const double d = box_sdf(x, b, &g) - pl.radius();
      if (d < dmin) {
        dmin = d;
        gmin = g;
      }
    }
    double dc;
    cost += cfun(dmin, eps, &dc);
    if (grad && dc != 0) {
      const Eigen::Matrix<double, 3, Eigen::Dynamic> Jx = (1 - p.s) * JO[p.frame - 1] + p.s * JO[p.frame];
      *grad += Jx.transpose() * (dc * gmin);
    }
  }
  return cost;
}

double min_clear_q(const Planner& pl, const Vec& q) {
  const Chain& c = pl.chain();
  const auto F = c.frames(q);
  double m = std::numeric_limits<double>::infinity();
  for (const Pt& p : body_points(c, q)) {
    const Eigen::Vector3d x = (1 - p.s) * F[p.frame - 1].translation() + p.s * F[p.frame].translation();
    if (x.norm() < 0.25) continue;
    for (const Box& b : pl.boxes()) m = std::min(m, box_sdf(x, b, nullptr) - pl.radius());
  }
  return m;
}
}  // namespace

double path_length(const std::vector<Vec>& path) {
  double L = 0;
  for (size_t i = 1; i < path.size(); ++i) L += (path[i] - path[i - 1]).norm();
  return L;
}

double smoothness_cost(const std::vector<Vec>& path) {
  double s = 0;
  for (size_t i = 1; i + 1 < path.size(); ++i) s += 0.5 * (path[i - 1] - 2 * path[i] + path[i + 1]).squaredNorm();
  return s;
}

std::vector<Vec> resample(const std::vector<Vec>& path, int n) {
  const double L = path_length(path);
  std::vector<Vec> out;
  size_t seg = 1;
  double acc = 0;  // arc length at path[seg-1]
  for (int k = 0; k < n; ++k) {
    const double target = L * k / (n - 1);
    while (seg < path.size() - 1 && acc + (path[seg] - path[seg - 1]).norm() < target) {
      acc += (path[seg] - path[seg - 1]).norm();
      ++seg;
    }
    const double sl = (path[seg] - path[seg - 1]).norm();
    const double u = sl > 0 ? std::clamp((target - acc) / sl, 0.0, 1.0) : 0.0;
    out.push_back(path[seg - 1] + u * (path[seg] - path[seg - 1]));
  }
  out.front() = path.front();
  out.back() = path.back();
  return out;
}

bool dense_valid(const Planner& pl, const std::vector<Vec>& path, double step) {
  for (size_t i = 1; i < path.size(); ++i) {
    const int k = std::max(1, static_cast<int>(std::ceil((path[i] - path[i - 1]).norm() / step)));
    for (int j = 0; j <= k; ++j)
      if (!pl.valid(path[i - 1] + (path[i] - path[i - 1]) * (static_cast<double>(j) / k))) return false;
  }
  return true;
}

double min_clearance(const Planner& pl, const std::vector<Vec>& path) {
  double m = std::numeric_limits<double>::infinity();
  for (size_t i = 1; i < path.size(); ++i) {
    const int k = std::max(1, static_cast<int>(std::ceil((path[i] - path[i - 1]).norm() / 0.01)));
    for (int j = 0; j <= k; ++j)
      m = std::min(m, min_clear_q(pl, path[i - 1] + (path[i] - path[i - 1]) * (static_cast<double>(j) / k)));
  }
  return m;
}

std::vector<Vec> chomp(const Planner& pl, const std::vector<Vec>& init, const ChompOptions& o, ChompStats* st) {
  const auto t0 = std::chrono::steady_clock::now();
  const Chain& c = pl.chain();
  std::vector<Vec> Q = resample(init, o.n_waypoints);
  const int N = static_cast<int>(Q.size()), m = N - 2, d = c.dof();
  // Smoothness metric on the interior points: A = K^T K (second differences), pentadiagonal.
  Mat K = Mat::Zero(N - 2, N);
  for (int j = 0; j < N - 2; ++j) {
    K(j, j) = 1;
    K(j, j + 1) = -2;
    K(j, j + 2) = 1;
  }
  const Mat Kint = K.middleCols(1, m);
  const Mat A = Kint.transpose() * Kint;
  const Eigen::LDLT<Mat> Ainv(A);
  auto total = [&](const std::vector<Vec>& P, double* sm, double* ob) {
    double s = smoothness_cost(P), b = 0;
    for (int i = 1; i < N - 1; ++i) b += obs_cost(pl, P[i], o.eps, nullptr);
    if (sm) *sm = s;
    if (ob) *ob = b;
    return o.w_smooth * s + o.w_obs * b;
  };
  ChompStats S;
  double cost = total(Q, &S.smooth_before, &S.obs_before);
  S.cost_before = cost;
  double eta = o.eta;
  for (int it = 0; it < o.max_iters; ++it) {
    S.iters = it + 1;
    // gradient: rows = interior waypoints, cols = joints
    Mat G(m, d);
    for (int i = 1; i < N - 1; ++i) {
      Vec gs = Vec::Zero(d);
      for (int j = i - 1; j <= i + 1; ++j) {  // accelerations a_j touching waypoint i
        if (j < 1 || j > N - 2) continue;
        const double coef = (j == i) ? -2.0 : 1.0;
        gs += coef * (Q[j - 1] - 2 * Q[j] + Q[j + 1]);
      }
      Vec go;
      obs_cost(pl, Q[i], o.eps, &go);
      G.row(i - 1) = (o.w_smooth * gs + o.w_obs * go).transpose();
    }
    const Mat step = Ainv.solve(G);
    double new_cost = cost;
    std::vector<Vec> Qn;
    for (int tries = 0; tries < 30; ++tries) {
      Qn = Q;
      for (int i = 1; i < N - 1; ++i) Qn[i] = c.clamp(Q[i] - eta * step.row(i - 1).transpose());
      new_cost = total(Qn, nullptr, nullptr);
      if (new_cost <= cost) break;
      eta *= 0.5;
    }
    if (new_cost > cost) break;  // no descent step found
    const double rel = (cost - new_cost) / std::max(cost, 1e-12);
    Q = Qn;
    cost = new_cost;
    if (rel < o.rel_tol) break;
  }
  S.cost_after = total(Q, &S.smooth_after, &S.obs_after);
  S.time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  if (st) *st = S;
  return Q;
}

}  // namespace kinmp

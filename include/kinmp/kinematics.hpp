// kinmp: header-only C++17 + Eigen kinematics for serial chains described by DH tables.
// Supports standard (Denavit-Hartenberg) and modified (Craig) DH, passive coupled joints
// (q_full = C * q_actuated, used for the palletizer's level-keeping wrist), a tool frame,
// forward kinematics, geometric Jacobian and damped-least-squares IK with joint limits.
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace kinmp {

using Vec = Eigen::VectorXd;
using Mat = Eigen::MatrixXd;
using Jac = Eigen::Matrix<double, 6, Eigen::Dynamic>;  // rows: vx vy vz wx wy wz (world frame)

enum class DH { Standard, Modified };

struct DHRow {
  double a;       // link length
  double alpha;   // link twist
  double d;       // link offset
  double offset;  // constant added to the joint angle
};

inline Eigen::Isometry3d dh_transform(DH conv, const DHRow& r, double q) {
  const double th = q + r.offset;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  if (conv == DH::Standard) {  // Rz(th) Tz(d) Tx(a) Rx(alpha)
    T.rotate(Eigen::AngleAxisd(th, Eigen::Vector3d::UnitZ()));
    T.translate(Eigen::Vector3d(r.a, 0, r.d));
    T.rotate(Eigen::AngleAxisd(r.alpha, Eigen::Vector3d::UnitX()));
  } else {  // Rx(alpha) Tx(a) Rz(th) Tz(d)
    T.rotate(Eigen::AngleAxisd(r.alpha, Eigen::Vector3d::UnitX()));
    T.translate(Eigen::Vector3d(r.a, 0, 0));
    T.rotate(Eigen::AngleAxisd(th, Eigen::Vector3d::UnitZ()));
    T.translate(Eigen::Vector3d(0, 0, r.d));
  }
  return T;
}

class Chain {
 public:
  Chain(std::string name, DH conv, std::vector<DHRow> rows, Mat coupling, Vec lower, Vec upper,
        Eigen::Isometry3d base = Eigen::Isometry3d::Identity(),
        Eigen::Isometry3d tool = Eigen::Isometry3d::Identity())
      : name_(std::move(name)), conv_(conv), rows_(std::move(rows)), C_(std::move(coupling)),
        lo_(std::move(lower)), hi_(std::move(upper)), base_(base), tool_(tool) {
    if (C_.rows() != static_cast<int>(rows_.size())) throw std::invalid_argument("coupling rows != DH rows");
    if (lo_.size() != C_.cols() || hi_.size() != C_.cols()) throw std::invalid_argument("limits size != dof");
  }

  const std::string& name() const { return name_; }
  int dof() const { return static_cast<int>(C_.cols()); }
  int num_joints() const { return static_cast<int>(rows_.size()); }
  const Vec& lower() const { return lo_; }
  const Vec& upper() const { return hi_; }
  Vec clamp(const Vec& q) const { return q.cwiseMax(lo_).cwiseMin(hi_); }
  bool within_limits(const Vec& q) const {
    return ((q - lo_).array() >= 0).all() && ((hi_ - q).array() >= 0).all();
  }

  // Frames 0..n (base, after each joint), then the tool frame (size n+2).
  std::vector<Eigen::Isometry3d> frames(const Vec& q) const {
    const Vec qf = C_ * q;
    std::vector<Eigen::Isometry3d> F;
    F.reserve(rows_.size() + 2);
    Eigen::Isometry3d T = base_;
    F.push_back(T);
    for (size_t i = 0; i < rows_.size(); ++i) {
      T = T * dh_transform(conv_, rows_[i], qf(static_cast<int>(i)));
      F.push_back(T);
    }
    F.push_back(T * tool_);
    return F;
  }

  Eigen::Isometry3d fk(const Vec& q) const { return frames(q).back(); }

  // Geometric Jacobian of the tool point w.r.t. the actuated joints, world frame.
  Jac jacobian(const Vec& q) const {
    const auto F = frames(q);
    const Eigen::Vector3d pe = F.back().translation();
    Jac Jf(6, num_joints());
    for (int i = 0; i < num_joints(); ++i) {
      // Standard DH: joint i rotates about z of frame i (before its transform, index i).
      // Modified DH: joint i rotates about z of frame i+1 (after its transform).
      const Eigen::Isometry3d& Fi = (conv_ == DH::Standard) ? F[i] : F[i + 1];
      const Eigen::Vector3d z = Fi.linear().col(2);
      const Eigen::Vector3d p = Fi.translation();
      Jf.block<3, 1>(0, i) = z.cross(pe - p);
      Jf.block<3, 1>(3, i) = z;
    }
    return Jf * C_;
  }

  // Position Jacobians (3 x dof) of every frame origin returned by frames(q).
  std::vector<Eigen::Matrix<double, 3, Eigen::Dynamic>> origin_jacobians(const Vec& q) const {
    const auto F = frames(q);
    std::vector<Eigen::Matrix<double, 3, Eigen::Dynamic>> out;
    for (size_t k = 0; k < F.size(); ++k) {
      Eigen::Matrix<double, 3, Eigen::Dynamic> Jf = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, num_joints());
      for (int i = 0; i < num_joints(); ++i) {
        const size_t ax = (conv_ == DH::Standard) ? i : i + 1;  // index of the frame holding joint i's axis
        if (k <= ax) continue;  // joint i does not move frame k (or k lies on its axis)
        Jf.col(i) = F[ax].linear().col(2).cross(F[k].translation() - F[ax].translation());
      }
      out.push_back(Jf * C_);
    }
    return out;
  }

  Vec random_q(std::mt19937& rng) const {
    Vec q(dof());
    for (int i = 0; i < dof(); ++i) q(i) = std::uniform_real_distribution<double>(lo_(i), hi_(i))(rng);
    return q;
  }

 private:
  std::string name_;
  DH conv_;
  std::vector<DHRow> rows_;
  Mat C_;
  Vec lo_, hi_;
  Eigen::Isometry3d base_, tool_;
};

// 6-vector pose error (position, rotation vector), world frame: target relative to current.
inline Eigen::Matrix<double, 6, 1> pose_error(const Eigen::Isometry3d& target, const Eigen::Isometry3d& cur) {
  Eigen::Matrix<double, 6, 1> e;
  e.head<3>() = target.translation() - cur.translation();
  const Eigen::AngleAxisd aa(target.linear() * cur.linear().transpose());
  e.tail<3>() = aa.angle() * aa.axis();
  return e;
}

struct IKOptions {
  int max_iters = 500;
  int max_attempts = 10;     // 1 = only the given seed; extra attempts restart from random seeds
  double tol_pos = 1e-4;     // m, Euclidean norm
  double tol_rot = 1e-3;     // rad
  double lambda = 0.05;      // damping
  double max_step = 0.3;     // rad per iteration (norm)
  std::array<bool, 6> mask{{true, true, true, true, true, true}};  // task rows used
  unsigned seed = 1;
};

struct IKResult {
  bool ok = false;
  int iters = 0;
  int attempts = 0;
  Vec q;
  double pos_err = 0, rot_err = 0;
};

// Damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 e, clamped to joint limits.
inline IKResult solve_ik(const Chain& c, const Eigen::Isometry3d& target, const Vec& seed,
                         const IKOptions& o = {}) {
  std::vector<int> rows;
  for (int i = 0; i < 6; ++i)
    if (o.mask[i]) rows.push_back(i);
  const int m = static_cast<int>(rows.size());
  std::mt19937 rng(o.seed);
  IKResult res;
  for (int att = 0; att < o.max_attempts; ++att) {
    Vec q = (att == 0) ? c.clamp(seed) : c.random_q(rng);
    res.attempts = att + 1;
    for (int it = 0; it < o.max_iters; ++it) {
      ++res.iters;
      const auto e6 = pose_error(target, c.fk(q));
      res.pos_err = e6.head<3>().norm();
      res.rot_err = 0;
      for (int r = 3; r < 6; ++r)
        if (o.mask[r]) res.rot_err += e6(r) * e6(r);
      res.rot_err = std::sqrt(res.rot_err);
      if (res.pos_err < o.tol_pos && res.rot_err < o.tol_rot) {
        res.ok = true;
        res.q = q;
        return res;
      }
      const Jac J6 = c.jacobian(q);
      Mat J(m, c.dof());
      Vec e(m);
      for (int k = 0; k < m; ++k) {
        J.row(k) = J6.row(rows[k]);
        e(k) = e6(rows[k]);
      }
      const Mat A = J * J.transpose() + o.lambda * o.lambda * Mat::Identity(m, m);
      Vec dq = J.transpose() * A.ldlt().solve(e);
      const double n = dq.norm();
      if (n > o.max_step) dq *= o.max_step / n;
      q = c.clamp(q + dq);
    }
  }
  res.q = Vec();
  return res;
}

// ---- The two chains used in this project ----

// Franka Panda, modified DH from Franka's published parameters; tool = flange + the
// attachment_site rotation of the MuJoCo Menagerie model (135 deg about z, quat 0.3827 0 0 0.9239).
inline Chain panda() {
  const double h = M_PI / 2;
  std::vector<DHRow> rows = {{0, 0, 0.333, 0},       {0, -h, 0, 0}, {0, h, 0.316, 0},     {0.0825, h, 0, 0},
                             {-0.0825, -h, 0.384, 0}, {0, h, 0, 0},  {0.088, h, 0, 0}};
  Vec lo(7), hi(7);
  lo << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
  hi << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
  Eigen::Isometry3d tool = Eigen::Isometry3d::Identity();
  tool.translate(Eigen::Vector3d(0, 0, 0.107));
  tool.rotate(Eigen::AngleAxisd(3 * M_PI / 4, Eigen::Vector3d::UnitZ()));
  return Chain("panda", DH::Modified, rows, Mat::Identity(7, 7), lo, hi, Eigen::Isometry3d::Identity(), tool);
}

// Custom 4-axis palletizer (standard DH): base yaw, shoulder pitch, elbow pitch, passive wrist
// pitch coupled as -(q2+q3) (the parallelogram linkage that keeps the tool level), wrist yaw.
// Tool z always points straight down. Dimensions are made up (not a real product).
inline Chain palletizer() {
  const double h = M_PI / 2;
  std::vector<DHRow> rows = {{0, h, 0.5, 0}, {1.0, 0, 0, 0}, {0.9, 0, 0, 0}, {0.2, h, 0, 0}, {0, 0, 0.25, 0}};
  Mat C = Mat::Zero(5, 4);
  C(0, 0) = 1;
  C(1, 1) = 1;
  C(2, 2) = 1;
  C(3, 1) = -1;
  C(3, 2) = -1;
  C(4, 3) = 1;
  Vec lo(4), hi(4);
  lo << -M_PI, -0.5, -2.5, -M_PI;
  hi << M_PI, 1.5, 0.0, M_PI;
  return Chain("palletizer", DH::Standard, rows, C, lo, hi);
}

// IK task rows for the palletizer: x, y, z and yaw (wz); roll/pitch are fixed by the linkage.
inline std::array<bool, 6> palletizer_mask() { return {{true, true, true, false, false, true}}; }

}  // namespace kinmp

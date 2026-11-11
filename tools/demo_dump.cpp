// Demo data for the README and project page (no ROS, no MuJoCo).
//   <out>/ik_targets.csv : the 1,000-target IK test of test_kinematics.cpp, one row per target
//                          (same random generator and seeds), for both chains
//   <out>/paths.csv      : the first of the 20 blocked Panda queries of test_chomp.cpp (same generator
//                          and planner seed): RRT-Connect raw, shortcut, shortcut then CHOMP, resampled
//                          to 50 waypoints, with joint angles and the tool position from FK
// Usage: kinmp_demo_dump <out_dir>
#include <fstream>
#include <iostream>
#include <random>
#include <string>

#include "kinmp/chomp.hpp"
#include "kinmp/kinematics.hpp"
#include "kinmp/scene.hpp"

using namespace kinmp;

int main(int argc, char** argv) {
  if (argc < 2) { std::cerr << "usage: kinmp_demo_dump <out_dir>\n"; return 1; }
  const std::string out = argv[1];

  std::ofstream ik(out + "/ik_targets.csv");
  ik << "chain,target,ok,attempts,iters,pos_err_m,rot_err_rad\n";
  for (const Chain& c : {panda(), palletizer()}) {
    IKOptions o;
    if (c.name() == "palletizer") o.mask = palletizer_mask();
    std::mt19937 rng(11);
    int ok = 0;
    for (int s = 0; s < 1000; ++s) {
      const Vec q_true = c.random_q(rng);
      const auto target = c.fk(q_true);
      const Vec seed = c.random_q(rng);
      o.seed = 1000 + s;
      const IKResult r = solve_ik(c, target, seed, o);
      ok += r.ok;
      ik << c.name() << ',' << s << ',' << r.ok << ',' << r.attempts << ',' << r.iters << ',' << r.pos_err << ','
         << r.rot_err << '\n';
    }
    std::cout << "IK " << c.name() << ": " << ok << " / 1000\n";
  }

  const Planner pl(panda(), demo_scene());
  std::mt19937 rng(5);
  Vec a, b;
  do {
    a = pl.chain().random_q(rng);
    b = pl.chain().random_q(rng);
  } while (!pl.valid(a) || !pl.valid(b) || dense_valid(pl, {a, b}));
  const PlanResult r = pl.plan(a, b, 5.0, 101);
  if (!r.ok) { std::cerr << "plan failed\n"; return 2; }
  ChompStats st;
  const auto ch = chomp(pl, r.path, ChompOptions{}, &st);
  std::ofstream p(out + "/paths.csv");
  p.precision(10);
  p << "variant,i,q1,q2,q3,q4,q5,q6,q7,x,y,z\n";
  auto dump = [&](const std::string& name, const std::vector<Vec>& path) {
    const auto rs = resample(path, 50);
    for (size_t i = 0; i < rs.size(); ++i) {
      p << name << ',' << i;
      for (int j = 0; j < 7; ++j) p << ',' << rs[i](j);
      const Eigen::Vector3d t = pl.chain().fk(rs[i]).translation();  // copy: fk() returns a temporary
      p << ',' << t.x() << ',' << t.y() << ',' << t.z() << '\n';
    }
    std::cout << name << ": length " << path_length(path) << " rad, smoothness "
              << smoothness_cost(resample(path, 50)) << ", clearance " << min_clearance(pl, path) << " m, valid "
              << dense_valid(pl, path) << '\n';
  };
  dump("raw", r.raw_path);
  dump("shortcut", r.path);
  dump("shortcut_chomp", ch);
  return 0;
}

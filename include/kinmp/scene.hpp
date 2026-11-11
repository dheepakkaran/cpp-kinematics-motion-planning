// The demo scene used by the planning test and the ROS 2 node: floor + a pillar + a shelf
// in front of the Panda (made-up boxes, metres, world frame = Panda base).
#pragma once
#include "kinmp/planner.hpp"

namespace kinmp {
inline std::vector<Box> demo_scene() {
  return {
      {{-2.0, -2.0, -1.0}, {2.0, 2.0, 0.0}},     // floor
      {{0.45, -0.10, 0.0}, {0.60, 0.10, 0.70}},  // pillar
      {{0.30, -0.60, 0.80}, {0.70, 0.60, 0.85}}, // overhead shelf
  };
}
inline Vec panda_home() {
  Vec q(7);
  q << 0, -0.785, 0, -2.356, 0, 1.571, 0.785;
  return q;
}
}  // namespace kinmp

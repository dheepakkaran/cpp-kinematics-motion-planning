// ROS 2 node: goal position on /kinmp/goal (std_msgs/Float64MultiArray [x, y, z], tool pointing
// down) -> IK -> OMPL plan + optimization -> trajectory executed by a fixed-rate loop thread
// (default 500 Hz, std::this_thread::sleep_until on steady_clock) that publishes
// /joint_states and counts missed deadlines. Status JSON on /kinmp/status (std_msgs/String).
// Kinematic simulation only; normal Linux kernel = soft real time.
#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "kinmp/scene.hpp"

namespace kinmp {

struct LoopStats {
  long iterations = 0;
  long missed = 0;            // iteration finished after its deadline (start + period)
  double max_wake_late_us = 0;
  std::vector<double> wake_late_us;  // per iteration while executing
};

class PlanningNode : public rclcpp::Node {
 public:
  explicit PlanningNode(const rclcpp::NodeOptions& o = rclcpp::NodeOptions());
  ~PlanningNode() override;
  LoopStats stats();
  double rate_hz() const { return rate_hz_; }

 private:
  void on_goal(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void loop();

  Planner planner_;
  double rate_hz_, vmax_, amax_;
  Eigen::Matrix3d down_R_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::mutex mu_;
  Vec q_;               // current commanded joints
  Trajectory traj_;     // active trajectory (empty = idle)
  bool new_traj_ = false;
  int goal_id_ = 0;
  LoopStats stats_;
  std::atomic<bool> run_{true};
  std::thread th_;
};

}  // namespace kinmp

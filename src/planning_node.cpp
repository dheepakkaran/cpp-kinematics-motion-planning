#include "kinmp/planning_node.hpp"

#include <chrono>
#include <sstream>

namespace kinmp {

PlanningNode::PlanningNode(const rclcpp::NodeOptions& o)
    : rclcpp::Node("kinmp_planner", o), planner_(panda(), demo_scene()) {
  rate_hz_ = declare_parameter("rate_hz", 500.0);
  vmax_ = declare_parameter("vmax", 1.0);
  amax_ = declare_parameter("amax", 2.0);
  q_ = panda_home();
  down_R_ = planner_.chain().fk(q_).linear();  // keep the home tool orientation (pointing down)
  js_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
  status_pub_ = create_publisher<std_msgs::msg::String>("kinmp/status", rclcpp::QoS(10).reliable());
  sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "kinmp/goal", 10, [this](std_msgs::msg::Float64MultiArray::SharedPtr m) { on_goal(m); });
  th_ = std::thread([this] { loop(); });
}

PlanningNode::~PlanningNode() {
  run_ = false;
  if (th_.joinable()) th_.join();
}

LoopStats PlanningNode::stats() {
  std::lock_guard<std::mutex> lk(mu_);
  return stats_;
}

void PlanningNode::on_goal(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  std::ostringstream js;
  const int id = ++goal_id_;
  js << "{\"id\": " << id << ", \"event\": \"planned\"";
  if (msg->data.size() != 3) {
    js << ", \"ok\": false, \"error\": \"need [x,y,z]\"}";
    std_msgs::msg::String s;
    s.data = js.str();
    status_pub_->publish(s);
    return;
  }
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.linear() = down_R_;
  target.translation() = Eigen::Vector3d(msg->data[0], msg->data[1], msg->data[2]);
  Vec q0;
  {
    std::lock_guard<std::mutex> lk(mu_);
    q0 = traj_.empty() ? q_ : traj_.sample(traj_.duration());
  }
  // IK; retry with new random seeds until the solution is collision-free.
  IKResult ik;
  bool ik_ok = false;
  for (int k = 0; k < 10 && !ik_ok; ++k) {
    IKOptions io;
    io.seed = 17 + k;
    ik = solve_ik(planner_.chain(), target, q0, io);
    ik_ok = ik.ok && planner_.valid(ik.q);
    if (!ik_ok) {
      std::mt19937 rng(100 + k);
      q0 = planner_.chain().random_q(rng);
    }
  }
  {
    std::lock_guard<std::mutex> lk(mu_);
    q0 = traj_.empty() ? q_ : traj_.sample(traj_.duration());
  }
  js << ", \"ik_ok\": " << (ik_ok ? "true" : "false");
  bool ok = false;
  if (ik_ok) {
    const PlanResult r = planner_.plan(q0, ik.q, 5.0, 1000 + id);
    js << ", \"plan_ok\": " << (r.ok ? "true" : "false") << ", \"plan_time_s\": " << r.plan_time_s
       << ", \"optimize_time_s\": " << r.optimize_time_s << ", \"len_raw\": " << r.len_raw
       << ", \"len_opt\": " << r.len_opt;
    if (r.ok) {
      Trajectory tr(r.path, vmax_, amax_);
      js << ", \"duration_s\": " << tr.duration();
      std::lock_guard<std::mutex> lk(mu_);
      traj_ = tr;
      new_traj_ = true;
      ok = true;
    }
  }
  js << ", \"ok\": " << (ok ? "true" : "false") << "}";
  std_msgs::msg::String s;
  s.data = js.str();
  status_pub_->publish(s);
}

void PlanningNode::loop() {
  using clk = std::chrono::steady_clock;
  const auto period = std::chrono::duration_cast<clk::duration>(std::chrono::duration<double>(1.0 / rate_hz_));
  auto next = clk::now() + period;
  clk::time_point t_start;
  bool executing = false;
  long it_exec = 0, missed_exec = 0;
  int pub_div = 0;
  sensor_msgs::msg::JointState js;
  for (int i = 1; i <= 7; ++i) js.name.push_back("joint" + std::to_string(i));
  while (run_ && rclcpp::ok()) {
    std::this_thread::sleep_until(next);
    const auto wake = clk::now();
    const double late_us = std::chrono::duration<double, std::micro>(wake - next).count();
    bool done = false;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (new_traj_) {
        new_traj_ = false;
        executing = true;
        t_start = wake;
        it_exec = missed_exec = 0;
      }
      if (executing) {
        const double t = std::chrono::duration<double>(wake - t_start).count();
        q_ = traj_.sample(t);
        ++stats_.iterations;
        ++it_exec;
        stats_.wake_late_us.push_back(late_us);
        stats_.max_wake_late_us = std::max(stats_.max_wake_late_us, late_us);
        if (t >= traj_.duration()) {
          done = true;
          executing = false;
          traj_ = Trajectory();
        }
      }
      js.position.assign(q_.data(), q_.data() + q_.size());
    }
    if (++pub_div >= 5 || done) {  // joint_states at rate/5 (100 Hz) to keep the loop light
      pub_div = 0;
      js.header.stamp = now();
      js_pub_->publish(js);
    }
    if (done) {
      std_msgs::msg::String s;
      s.data = "{\"event\": \"done\", \"iterations\": " + std::to_string(it_exec) +
               ", \"missed\": " + std::to_string(missed_exec) + "}";
      status_pub_->publish(s);
    }
    next += period;
    if (clk::now() > next) {  // this iteration ran past its deadline
      if (executing || done) {
        std::lock_guard<std::mutex> lk(mu_);
        ++stats_.missed;
        ++missed_exec;
      }
      while (next < clk::now()) next += period;  // skip the lost slots, keep the grid
    }
  }
}

}  // namespace kinmp

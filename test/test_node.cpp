// In-process integration test: the real PlanningNode + a client node on one
// MultiThreadedExecutor, talking over ROS topics. Sends 3 goal positions around the pillar,
// waits for each "done", checks the final joint state reaches the goal, writes results/rt_loop.json.
#include <gtest/gtest.h>

#include <algorithm>
#include <condition_variable>
#include <fstream>
#include <iomanip>

#include "kinmp/planning_node.hpp"

using namespace std::chrono_literals;

TEST(PlanningNode, ThreeGoalsPlannedAndExecutedInRealTimeLoop) {
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<kinmp::PlanningNode>();
    auto client = std::make_shared<rclcpp::Node>("test_client");
    auto pub = client->create_publisher<std_msgs::msg::Float64MultiArray>("kinmp/goal", 10);
    std::mutex mu;
    std::condition_variable cv;
    std::vector<std::string> status;
    sensor_msgs::msg::JointState last;
    auto ssub = client->create_subscription<std_msgs::msg::String>(
        "kinmp/status", rclcpp::QoS(10).reliable(), [&](std_msgs::msg::String::SharedPtr m) {
          std::lock_guard<std::mutex> lk(mu);
          status.push_back(m->data);
          cv.notify_all();
        });
    auto jsub = client->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS(), [&](sensor_msgs::msg::JointState::SharedPtr m) {
          std::lock_guard<std::mutex> lk(mu);
          last = *m;
        });
    rclcpp::executors::MultiThreadedExecutor ex(rclcpp::ExecutorOptions(), 2);
    ex.add_node(node);
    ex.add_node(client);
    std::thread spin([&] { ex.spin(); });
    for (int i = 0; i < 200 && pub->get_subscription_count() == 0; ++i) std::this_thread::sleep_for(10ms);

    const std::vector<std::array<double, 3>> goals = {{0.35, -0.40, 0.40}, {0.35, 0.40, 0.40}, {0.40, -0.35, 0.25}};
    const kinmp::Chain c = kinmp::panda();
    double max_goal_err = 0;
    int done = 0, planned_ok = 0;
    for (const auto& g : goals) {
      std_msgs::msg::Float64MultiArray m;
      m.data = {g[0], g[1], g[2]};
      size_t before;
      {
        std::lock_guard<std::mutex> lk(mu);
        before = status.size();
      }
      pub->publish(m);
      std::unique_lock<std::mutex> lk(mu);
      // expect "planned" then "done"
      ASSERT_TRUE(cv.wait_for(lk, 15s, [&] { return status.size() >= before + 1; }));
      if (status[before].find("\"ok\": true") == std::string::npos) {
        ADD_FAILURE() << status[before];
        continue;
      }
      ++planned_ok;
      ASSERT_TRUE(cv.wait_for(lk, 30s, [&] { return status.size() >= before + 2; })) << status[before];
      ++done;
      lk.unlock();
      std::this_thread::sleep_for(50ms);  // let the last joint_states arrive
      lk.lock();
      ASSERT_EQ(last.position.size(), 7u);
      kinmp::Vec q(7);
      for (int i = 0; i < 7; ++i) q(i) = last.position[i];
      max_goal_err = std::max(max_goal_err, (c.fk(q).translation() - Eigen::Vector3d(g[0], g[1], g[2])).norm());
    }
    const kinmp::LoopStats st = node->stats();
    ex.cancel();
    spin.join();

    auto lat = st.wake_late_us;
    std::sort(lat.begin(), lat.end());
    auto p = [&](double f) { return lat.empty() ? 0.0 : lat[std::min(lat.size() - 1, size_t(f * lat.size()))]; };
    double mean = 0;
    for (double x : lat) mean += x;
    mean = lat.empty() ? 0 : mean / lat.size();
    std::ofstream f(std::string(RESULTS_DIR) + "/rt_loop.json");
    f << std::setprecision(6) << "{\n  \"note\": \"soft real time: normal Linux kernel, sleep_until on steady_clock, kinematic simulation\",\n"
      << "  \"rate_hz\": " << node->rate_hz() << ",\n  \"goals\": " << goals.size() << ",\n  \"planned_ok\": " << planned_ok
      << ",\n  \"executed\": " << done << ",\n  \"final_tool_pos_err_m_max\": " << max_goal_err
      << ",\n  \"loop_iterations_executing\": " << st.iterations << ",\n  \"missed_deadlines\": " << st.missed
      << ",\n  \"wake_late_us_mean\": " << mean << ",\n  \"wake_late_us_p50\": " << p(0.5) << ",\n  \"wake_late_us_p99\": " << p(0.99)
      << ",\n  \"wake_late_us_max\": " << st.max_wake_late_us << ",\n  \"status_messages\": [\n";
    for (size_t i = 0; i < status.size(); ++i) f << "    " << status[i] << (i + 1 < status.size() ? ",\n" : "\n");
    f << "  ]\n}\n";
    EXPECT_EQ(done, 3);
    EXPECT_LT(max_goal_err, 1e-3);
    EXPECT_GT(st.iterations, 0);
  }
  rclcpp::shutdown();
}

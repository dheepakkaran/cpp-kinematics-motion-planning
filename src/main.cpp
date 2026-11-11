#include <rclcpp/rclcpp.hpp>

#include "kinmp/planning_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor ex;
  auto n = std::make_shared<kinmp::PlanningNode>();
  ex.add_node(n);
  ex.spin();
  rclcpp::shutdown();
  return 0;
}

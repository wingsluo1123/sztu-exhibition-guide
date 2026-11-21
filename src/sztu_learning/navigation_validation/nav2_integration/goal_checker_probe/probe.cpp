// Replay recorded poses through the installed official goal checker. No motor API.
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "nav2_controller/plugins/simple_goal_checker.hpp"

geometry_msgs::msg::Pose pose(double x, double y, double yaw) {
  geometry_msgs::msg::Pose p;
  p.position.x = x; p.position.y = y;
  p.orientation.z = std::sin(yaw / 2.); p.orientation.w = std::cos(yaw / 2.);
  return p;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  if (argc != 4) {std::cerr << "goal_x goal_y goal_yaw required\n"; return 2;}
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("goal_checker_trace_probe");
  nav2_controller::SimpleGoalChecker latched, simultaneous;
  for (const auto & name : {std::string("latched"), std::string("simultaneous")}) {
    node->declare_parameter(name + ".xy_goal_tolerance", .15);
    node->declare_parameter(name + ".yaw_goal_tolerance", .10);
    node->declare_parameter(name + ".stateful", name == "latched");
  }
  latched.initialize(node, "latched", nullptr);
  simultaneous.initialize(node, "simultaneous", nullptr);
  const auto goal = pose(std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3]));
  geometry_msgs::msg::Twist speed;
  std::string line;
  int i = 0;
  while (std::getline(std::cin, line)) {
    std::istringstream row(line); double x, y, yaw;
    if (!(row >> x >> y >> yaw)) {return 3;}
    const auto current = pose(x, y, yaw);
    const bool old_result = latched.isGoalReached(current, goal, speed);
    const bool new_result = simultaneous.isGoalReached(current, goal, speed);
    std::cout << i++ << " " << old_result << " " << new_result << " "
              << std::hypot(x - goal.position.x, y - goal.position.y) << "\n";
  }
  rclcpp::shutdown();
  return 0;
}

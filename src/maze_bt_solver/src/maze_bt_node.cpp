#include <behaviortree_cpp_v3/bt_factory.h>
#include "maze_bt_solver/robot.hpp"
#include "maze_bt_solver/bt_nodes.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("maze_bt_solver");

  Robot robot(node);

  BT::BehaviorTreeFactory factory;

  double goal_x = 2.0;
  double goal_y = 2.0;

  // Register nodes using lambdas that capture 'robot' and goal
  factory.registerNodeType<MoveForward>("MoveForward"); // keep if simple constructor
  factory.registerNodeType<Turn>("Turn");

  factory.registerSimpleCondition(
    "GoalReached",
    [&robot, goal_x, goal_y]() -> BT::NodeStatus
    {
      auto p = robot.getPose();
      double dx = goal_x - p.x;
      double dy = goal_y - p.y;
      double dist = std::sqrt(dx*dx + dy*dy);
      return dist < 0.2 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });

  auto tree = factory.createTreeFromFile("behavior_trees/maze_solver_ros.xml");

  rclcpp::Rate rate(5);
  while (rclcpp::ok())
  {
      rclcpp::spin_some(node);
      tree.tickRoot();
      rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

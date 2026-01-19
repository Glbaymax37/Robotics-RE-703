#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace BT;

/* ================= CHECK OBSTACLE ================= */
class CheckObstacle : public ConditionNode
{
public:
  CheckObstacle(const std::string& name, const NodeConfiguration& config)
  : ConditionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

    sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&CheckObstacle::scanCallback, this, std::placeholders::_1));
  }

  static PortsList providedPorts() { return {}; }

  NodeStatus tick() override
  {
    if (!scan_received_) return NodeStatus::FAILURE;

    if (front_dist_ < 0.8)
    {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Obstacle detected: %.2f m", front_dist_);
      return NodeStatus::SUCCESS;
    }

    return NodeStatus::FAILURE;
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    int center = msg->ranges.size() / 2;
    int window = 20;

    float min_dist = msg->range_max;
    for (int i = center - window; i <= center + window; i++)
    {
      if (std::isfinite(msg->ranges[i]))
        min_dist = std::min(min_dist, msg->ranges[i]);
    }

    front_dist_ = min_dist;
    scan_received_ = true;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  bool scan_received_{false};
  float front_dist_{999.0};
};

/* ================= MOVE FORWARD ================= */
class MoveForward : public SyncActionNode
{
public:
  MoveForward(const std::string& name, const NodeConfiguration& config)
  : SyncActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

  static PortsList providedPorts() { return {}; }

  NodeStatus tick() override
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.5;
    cmd.angular.z = 0.0;
    pub_->publish(cmd);
    return NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

/* ================= ROTATE ================= */
class Rotate : public SyncActionNode
{
public:
  Rotate(const std::string& name, const NodeConfiguration& config)
  : SyncActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

  static PortsList providedPorts() { return {}; }

  NodeStatus tick() override
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 0.8;
    pub_->publish(cmd);
    return NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

/* ================= MAIN ================= */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bt_controller");

  BehaviorTreeFactory factory;
  factory.registerNodeType<CheckObstacle>("CheckObstacle");
  factory.registerNodeType<MoveForward>("MoveForward");
  factory.registerNodeType<Rotate>("Rotate");

  auto blackboard = Blackboard::create();
  blackboard->set("node", node);

  std::string pkg_path =
    ament_index_cpp::get_package_share_directory("bt_controller");

  auto tree = factory.createTreeFromFile(
    pkg_path + "/bt_xml/simple_bt.xml", blackboard);

  StdCoutLogger logger(tree);

  rclcpp::Rate rate(10);
  while (rclcpp::ok())
  {
    tree.tickRoot();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
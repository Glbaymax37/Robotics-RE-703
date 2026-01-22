#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <mutex>

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

class Robot
{
public:
  Robot(rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/diff_cont/odom", 10,
      std::bind(&Robot::odomCallback, this, std::placeholders::_1));

    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&Robot::scanCallback, this, std::placeholders::_1));
  }

  Pose2D getPose()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return pose_;
  }

  bool isObstacleAhead(double threshold = 0.5)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ranges_.empty()) return false;

    // check +-15 degrees
    int center = ranges_.size() / 2;
    int window = 15;
    for (int i = center - window; i <= center + window; ++i)
    {
      if (ranges_[i] < threshold) return true;
    }
    return false;
  }

  void move(double linear, double angular)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;
    cmd_pub_->publish(msg);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_.x = msg->pose.pose.position.x;
    pose_.y = msg->pose.pose.position.y;
    // Yaw extraction
    double siny_cosp = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                              msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                                    msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    pose_.theta = std::atan2(siny_cosp, cosy_cosp);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ranges_ = msg->ranges;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::vector<float> ranges_;
  Pose2D pose_;
  std::mutex mutex_;
};

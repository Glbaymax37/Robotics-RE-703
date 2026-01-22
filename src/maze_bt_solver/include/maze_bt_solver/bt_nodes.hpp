#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include "robot.hpp"

// Condition: Goal reached
class GoalReached : public BT::ConditionNode
{
public:
  GoalReached(const std::string & name, Robot & robot, double goal_x, double goal_y)
  : BT::ConditionNode(name, {}), robot_(robot), goal_x_(goal_x), goal_y_(goal_y) {}

  BT::NodeStatus tick() override
  {
    auto p = robot_.getPose();
    double dx = goal_x_ - p.x;
    double dy = goal_y_ - p.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    return dist < 0.2 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  Robot & robot_;
  double goal_x_, goal_y_;
};

// Action: Move forward (if no obstacle)
class MoveForward : public BT::SyncActionNode
{
public:
  MoveForward(const std::string & name, Robot & robot, double speed=0.2)
  : BT::SyncActionNode(name, {}), robot_(robot), speed_(speed) {}

  BT::NodeStatus tick() override
  {
    if (robot_.isObstacleAhead())
      return BT::NodeStatus::FAILURE;

    robot_.move(speed_, 0.0);
    return BT::NodeStatus::SUCCESS;
  }

private:
  Robot & robot_;
  double speed_;
};

// Action: Turn in place
class Turn : public BT::SyncActionNode
{
public:
  Turn(const std::string & name, Robot & robot, double angular=0.5)
  : BT::SyncActionNode(name, {}), robot_(robot), angular_(angular) {}

  BT::NodeStatus tick() override
  {
    robot_.move(0.0, angular_);
    return BT::NodeStatus::SUCCESS;
  }

private:
  Robot & robot_;
  double angular_;
};

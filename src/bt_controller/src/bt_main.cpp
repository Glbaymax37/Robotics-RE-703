#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <algorithm>
#include <cmath>

using std::placeholders::_1;

/* =========================
   SENSOR DATA
========================= */
struct SensorData
{
    float front = 10.0;
    float left = 10.0;
    float right = 10.0;
    float back = 10.0;

    bool front_blocked = false; // hysteresis result
    bool is_turning = false;

    std::string turn_direction = "none";

    int turn_cooldown = 0; // anti turn spam
};

/* =========================
   CONDITION: ALL BLOCKED
========================= */
class AllDirectionsBlocked : public BT::ConditionNode
{
public:
    AllDirectionsBlocked(const std::string &name,
                         const BT::NodeConfiguration &config,
                         SensorData *data)
        : BT::ConditionNode(name, config), data_(data) {}

    BT::NodeStatus tick() override
    {
        const float t = 0.35;
        return (data_->front < t &&
                data_->left < t &&
                data_->right < t &&
                data_->back < t)
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts() { return {}; }

private:
    SensorData *data_;
};

/* =========================
   CONDITION: FRONT BLOCKED
========================= */
class FrontBlocked : public BT::ConditionNode
{
public:
    FrontBlocked(const std::string &name,
                 const BT::NodeConfiguration &config,
                 SensorData *data)
        : BT::ConditionNode(name, config), data_(data) {}

    BT::NodeStatus tick() override
    {
        if (data_->is_turning)
            return BT::NodeStatus::FAILURE;
        if (data_->turn_cooldown > 0)
            return BT::NodeStatus::FAILURE;

        return data_->front_blocked
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts() { return {}; }

private:
    SensorData *data_;
};

/* =========================
   ACTION: TURN (CONFIDENT)
========================= */
class TurnToBestDirection : public BT::StatefulActionNode
{
public:
    TurnToBestDirection(const std::string &name,
                        const BT::NodeConfiguration &config,
                        SensorData *data,
                        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub)
        : BT::StatefulActionNode(name, config),
          data_(data), pub_(pub) {}

    BT::NodeStatus onStart() override
    {
        if (data_->left > 1.2)
            data_->turn_direction = "left";
        else if (data_->front > 1.2)
            data_->turn_direction = "front";
        else if (data_->right > 1.2)
            data_->turn_direction = "right";
        else
            data_->turn_direction = "back";

        data_->is_turning = true;

        phase_ = 0;
        ticks_ = 0;

        RCLCPP_INFO(rclcpp::get_logger("TURN"),
                    "TURN -> %s", data_->turn_direction.c_str());

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        geometry_msgs::msg::Twist cmd;

        /* ---- PHASE 0: ROTATE ---- */
        if (phase_ == 0)
        {
            if (data_->turn_direction == "left")
                cmd.angular.z = 1.0;
            else if (data_->turn_direction == "right")
                cmd.angular.z = -1.0;
            else if (data_->turn_direction == "back")
                cmd.angular.z = 1.6;
            else
                phase_ = 1;

            // CONFIDENT exit condition
            if (ticks_ > 8 && data_->front > 1.0)
            {
                phase_ = 1;
                ticks_ = 0;
            }
        }

        /* ---- PHASE 1: CLEARANCE ---- */
        else if (phase_ == 1)
        {
            cmd.linear.x = 0.25;

            if (ticks_ > 15 || data_->front < 0.3)
                phase_ = 2;
        }

        pub_->publish(cmd);
        ticks_++;

        /* ---- DONE ---- */
        if (phase_ == 2 || ticks_ > 150)
        {
            geometry_msgs::msg::Twist stop;
            pub_->publish(stop);

            data_->is_turning = false;
            data_->turn_cooldown = 10; // 1 detik

            RCLCPP_INFO(rclcpp::get_logger("TURN"), "DONE");
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        data_->is_turning = false;
    }

    static BT::PortsList providedPorts() { return {}; }

private:
    SensorData *data_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

    int phase_ = 0;
    int ticks_ = 0;
};

/* =========================
   ACTION: FOLLOW RIGHT WALL
========================= */
class FollowRightWall : public BT::SyncActionNode
{
public:
    FollowRightWall(const std::string &name,
                    const BT::NodeConfiguration &config,
                    SensorData *data,
                    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub)
        : BT::SyncActionNode(name, config),
          data_(data), pub_(pub) {}

    BT::NodeStatus tick() override
    {
        if (data_->is_turning)
            return BT::NodeStatus::SUCCESS;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.18;

        // Right wall hilang -> lurus
        if (data_->right > 2.0)
        {
            cmd.angular.z = 0.0;
            pub_->publish(cmd);
            return BT::NodeStatus::SUCCESS;
        }

        float error = 0.55 - data_->right;

        if (std::abs(error) < 0.2)
            integral_ += error * 0.05;
        else
            integral_ = 0.0;

        integral_ = std::clamp(integral_, -0.3f, 0.3f);

        cmd.angular.z = error * 1.2 + integral_;
        cmd.angular.z = std::clamp(cmd.angular.z, -0.8, 0.8);

        if (data_->front < 0.8)
            cmd.linear.x = 0.1;

        pub_->publish(cmd);
        return BT::NodeStatus::SUCCESS;
    }

    static BT::PortsList providedPorts() { return {}; }

private:
    SensorData *data_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    float integral_ = 0.0;
};

/* =========================
   MAIN NODE
========================= */
class MazeBT : public rclcpp::Node
{
public:
    MazeBT() : Node("maze_bt")
    {
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MazeBT::scanCallback, this, _1));

        factory_.registerBuilder<AllDirectionsBlocked>(
            "AllDirectionsBlocked",
            [this](auto n, auto c)
            { return std::make_unique<AllDirectionsBlocked>(n, c, &data_); });

        factory_.registerBuilder<FrontBlocked>(
            "FrontBlocked",
            [this](auto n, auto c)
            { return std::make_unique<FrontBlocked>(n, c, &data_); });

        factory_.registerBuilder<TurnToBestDirection>(
            "TurnToBestDirection",
            [this](auto n, auto c)
            { return std::make_unique<TurnToBestDirection>(n, c, &data_, cmd_pub_); });

        factory_.registerBuilder<FollowRightWall>(
            "FollowRightWall",
            [this](auto n, auto c)
            { return std::make_unique<FollowRightWall>(n, c, &data_, cmd_pub_); });

        auto pkg = ament_index_cpp::get_package_share_directory("bt_controller");
        tree_ = factory_.createTreeFromFile(pkg + "/bt/maze_tree.xml");

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MazeBT::tickTree, this));
    }

private:
    void scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        data_.front = getRange(msg, -20, 20);
        data_.right = getRange(msg, -90, -70);
        data_.left = getRange(msg, 70, 90);
        data_.back = getRange(msg, 170, 190);

        // HYSTERESIS FRONT BLOCK
        if (data_.front < 0.45)
            data_.front_blocked = true;
        else if (data_.front > 0.75)
            data_.front_blocked = false;
    }

    float getRange(sensor_msgs::msg::LaserScan::SharedPtr msg,
                   int start_deg, int end_deg)
    {
        float min_r = 10.0;
        int total = msg->ranges.size();

        for (int d = start_deg; d <= end_deg; d++)
        {
            int idx = (total / 2 + d * total / 360) % total;
            if (idx < 0)
                idx += total;

            float r = msg->ranges[idx];
            if (std::isfinite(r) && r > 0.1)
                min_r = std::min(min_r, r);
        }
        return min_r;
    }

    void tickTree()
    {
        if (data_.turn_cooldown > 0)
            data_.turn_cooldown--;

        tree_.tickRoot();
    }

    SensorData data_;
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

/* ========================= */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MazeBT>());
    rclcpp::shutdown();
    return 0;
}

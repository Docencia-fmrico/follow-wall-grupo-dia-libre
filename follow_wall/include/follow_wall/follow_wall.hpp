#ifndef FOLLOW_WALL_HPP_
#define FOLLOW_WALL_HPP_

#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace follow_wall
{

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class FollowWallLifeCycle : public rclcpp_lifecycle::LifecycleNode
{
    public:

        FollowWallLifeCycle();

        CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

        CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

        CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

        CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

        CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

        CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

        void do_work();

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr speed_pub_;

        bool objectLeft;
        bool objectRight;
        bool objectCenter;
        int state_;
        rclcpp::Time last_time_;

        const float SWEEPING_RANGE = 200;
        const float OBJECT_LIMIT = 0.75;

        // to understand how these functions work check this image
        // https://imgur.com/a/6N0uFbl

        bool get_object_right(sensor_msgs::msg::LaserScan::SharedPtr laser_data);
        bool get_object_center(sensor_msgs::msg::LaserScan::SharedPtr laser_data);
        bool get_object_left(sensor_msgs::msg::LaserScan::SharedPtr laser_data);

        void laser_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);

};
}

#endif //FOLLOW_WALL_HPP_
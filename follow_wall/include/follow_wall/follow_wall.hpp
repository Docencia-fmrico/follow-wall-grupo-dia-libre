#ifndef FOLLOW_WALL_HPP_
#define FOLLOW_WALL_HPP_

#include <memory>
#include <cmath>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#define RIGHT -1
#define LEFT 1

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

        float distance_to_left_;
        float distance_to_center_;
        float prev_left_;

        int state_;
        rclcpp::Time last_time_;
        bool is_turning_;
        int turn_to_;

        const float SWEEPING_RANGE = 5;
        const float OBJECT_LIMIT = 0.75;

        const float LEFT_DETECTION_ANGLE = 1.90;

        // to understand how these functions work check this image
        // https://imgur.com/a/6N0uFbl

        geometry_msgs::msg::Twist turn(int direction);
        
        int get_left_lecture(sensor_msgs::msg::LaserScan::SharedPtr laser_data);

        float get_object_center(sensor_msgs::msg::LaserScan::SharedPtr laser_data);
        float get_object_left(sensor_msgs::msg::LaserScan::SharedPtr laser_data);

        void laser_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);

};
}

#endif //FOLLOW_WALL_HPP_
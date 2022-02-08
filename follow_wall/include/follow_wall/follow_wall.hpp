#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

class FollowWallLifeCycle : public rclcpp_lifecycle::LifecycleNode
{
    public:

        FollowWallLifeCycle()
        : rclcpp_lifecycle::LifecycleNode("follow_wall_lifecycle")
        {
            //el topic de velocidad es /nav_vel y el tipo de mensaje es geometry_msgs/msg/Twist B)
            laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_raw", 10, std::bind(&FollowWallLifeCycle::laser_cb, this, _1));
            speed_pub_ = create_publisher<geometry_msgs::msg::Twist>("/nav_vel", 10); //preguntar que poner aqui
        }

        using CallbackReturnT =
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

        CallbackReturnT on_configure(const rclcpp_lifecycle::State & state)
        {
            RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());
            
            return CallbackReturnT::SUCCESS;
        }

        CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) 
        {
            RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());
            return CallbackReturnT::SUCCESS;
        }

        CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) 
        {
            RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());
            return CallbackReturnT::SUCCESS;
        }

        CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state) 
        {
            RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());
            return CallbackReturnT::SUCCESS;
        }

        CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state) 
        {
            RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
            return CallbackReturnT::SUCCESS;
        }

            CallbackReturnT on_error(const rclcpp_lifecycle::State & state) 
        {
            RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
            return CallbackReturnT::SUCCESS;
        }

        void do_work();

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr speed_pub_;
        bool objectLeft;
        bool objectRight;
        bool objectCenter;

        void laser_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);


};
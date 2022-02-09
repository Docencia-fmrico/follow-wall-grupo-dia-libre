#include "follow_wall/follow_wall.hpp"
#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


namespace follow_wall
{
  using rcl_interfaces::msg::ParameterType;
  using std::placeholders::_1;
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  FollowWallLifeCycle::FollowWallLifeCycle(): rclcpp_lifecycle::LifecycleNode("follow_wall_lifecycle")
  {
  }

  CallbackReturnT
  FollowWallLifeCycle::on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());

    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan_raw", 10, std::bind(&FollowWallLifeCycle::laser_cb, this, _1));
    speed_pub_ = create_publisher<geometry_msgs::msg::Twist>("/nav_vel", 10); 
    state_ = 0;
    is_turning_ = false;
            
    return CallbackReturnT::SUCCESS;
  }
  
  CallbackReturnT
  FollowWallLifeCycle::on_activate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());
    speed_pub_->on_activate();
    return CallbackReturnT::SUCCESS;
  }
  CallbackReturnT
  FollowWallLifeCycle::on_deactivate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT
  FollowWallLifeCycle::on_cleanup(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT
  FollowWallLifeCycle::on_shutdown(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT
  FollowWallLifeCycle::on_error(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }

  geometry_msgs::msg::Twist 
  FollowWallLifeCycle::turn(int direction){
    geometry_msgs::msg::Twist msg;

    if (direction == RIGHT){
      msg.linear.x = 0;
      msg.angular.z = -0.25;
    }
    else{
      msg.linear.x = 0.25;
      msg.angular.z = 0.25;
    }
    return msg;
  }

  void  
  FollowWallLifeCycle::do_work() {
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }

    geometry_msgs::msg::Twist cmd;
    RCLCPP_INFO(get_logger(), "Measure Center [%f]", distance_to_center_);
    RCLCPP_INFO(get_logger(), "Measure Left [%f]", distance_to_left_);

    
    if (!state_)
    {
        if (distance_to_center_ > OBJECT_LIMIT && !is_turning_)
        {
          cmd.linear.x = 0.25;
          cmd.angular.z = 0;
        }
        else
        {
          is_turning_ = true;
          if (distance_to_left_ > (OBJECT_LIMIT - 0.01) && distance_to_left_ > (OBJECT_LIMIT + 0.01)){
            cmd = turn(RIGHT);
          }
          else{
            state_ = 1;
            is_turning_ = false;
          }
        }
    }
    else
    {  
      if (is_turning_){
        if (turn_to_ == LEFT){
          if (distance_to_left_ > (OBJECT_LIMIT - 0.01) && distance_to_left_ > (OBJECT_LIMIT + 0.01)){
            cmd = turn(LEFT);
          }
          else{
            is_turning_ = false;
          }
        }
        else{
          if (distance_to_left_ > (OBJECT_LIMIT - 0.01) && distance_to_left_ > (OBJECT_LIMIT + 0.01)){
            cmd = turn(RIGHT);
          }
          else{
            is_turning_ = false;
          }

        }
      }
      if (distance_to_center_ > OBJECT_LIMIT + 1 && distance_to_left_ > (OBJECT_LIMIT + 1))
      {
        is_turning_ = true;
        turn_to_ = LEFT;
      }
      else if(distance_to_center_ < OBJECT_LIMIT)
      {
        is_turning_ = true;
        turn_to_ = RIGHT;
      }
      else
      {
        is_turning_ = false;
        cmd.linear.x = 0.25;
        cmd.angular.z = 0;
      }

    }

    speed_pub_->publish(cmd);
    //RCLCPP_INFO(get_logger(), "State [%d]", state_);
      
  }
  int
  FollowWallLifeCycle::get_left_lecture(sensor_msgs::msg::LaserScan::SharedPtr laser_data){
    return (M_PI/2 - laser_data->angle_min)/laser_data->angle_increment;
  }

  float
  FollowWallLifeCycle::get_object_center(sensor_msgs::msg::LaserScan::SharedPtr laser_data){
    return laser_data->ranges[laser_data->ranges.size()/2];
  }

  float
  FollowWallLifeCycle::get_object_left(sensor_msgs::msg::LaserScan::SharedPtr laser_data){
    return laser_data->ranges[get_left_lecture(laser_data)];
  }
  
  void
  FollowWallLifeCycle::laser_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {    
    distance_to_left_ = get_object_left(msg);
    distance_to_center_ = get_object_center(msg);
  }

}
  
  // here we can define 3 zones in the msg->ranges : left center and right.
  // according to the value on those 3 zones, we can update a boolean for each:
  // objectLeft = true/false, objectright, objectcenter...
  // we can define those variables as private and update them in callback to use them in dowork.
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
    turn_to_ = 1;
    prev_left_ = 0;
    prev_mean_ = 0;
    min_dist_ = 25.0;
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
      msg.angular.z = -0.15;
    }
    else{
      msg.linear.x = 0;
      msg.angular.z = 0.15;
    }
    return msg;
  }

  void  
  FollowWallLifeCycle::do_work() {
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }

    geometry_msgs::msg::Twist cmd;
    //RCLCPP_INFO(get_logger(), "Measure Center [%f]", distance_to_center_);
    //RCLCPP_INFO(get_logger(), "Measure Left [%f]", distance_to_left_);

    if (distance_to_center_ == 0 || distance_to_left_ == 0){ // at the beggining laser seems empty so avoid that
      return;
    }
    
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
        cmd = turn(turn_to_*RIGHT);
        if (distance_to_left_ > OBJECT_LIMIT/2)
        {
          tend_mean_ += distance_to_left_;
          tend_it_++;
        }
        
        if (tend_it_ == MAX_IT)
        {
          //RCLCPP_INFO(get_logger(), "Measure Left [%f]", distance_to_left_);
          //RCLCPP_INFO(get_logger(), "Min Left [%f]", min_dist_);
          //RCLCPP_INFO(get_logger(), "Prev Mean [%f]", prev_mean_);
          //RCLCPP_INFO(get_logger(), "Tend Mean [%f]", tend_mean_/MAX_IT);
          tend_it_ = 0;
          if (!prev_mean_)
          {
            prev_mean_ = tend_mean_ / MAX_IT;
          }
          else
          {
            if (prev_mean_ > tend_mean_ / MAX_IT)
            {
              prev_mean_ = tend_mean_ / MAX_IT;
            }
            else
            {
              turn_to_ *= -1;
              cmd = turn(turn_to_*RIGHT);
              if (distance_to_left_ <= min_dist_ + 0.05)
              {
                state_ = 2;
                is_turning_ = false;
              }
            }
          }
          tend_mean_ = 0;
        }
        if (min_dist_ > tend_mean_/MAX_IT)
        {
          min_dist_ = distance_to_left_;
        }
      }
    }



    speed_pub_->publish(cmd);
    //RCLCPP_INFO(get_logger(), "State [%d]", state_);
      
  }
  float
  FollowWallLifeCycle::get_left_lecture(sensor_msgs::msg::LaserScan::SharedPtr laser_data){
    return laser_data->ranges.size()/2 + (LEFT_DETECTION_ANGLE)/laser_data->angle_increment;
  }

  float
  FollowWallLifeCycle::get_object_center(sensor_msgs::msg::LaserScan::SharedPtr laser_data){
    int start = laser_data->ranges.size()/2 - SWEEPING_RANGE/2;
    int end = laser_data->ranges.size()/2 + SWEEPING_RANGE/2;
    float avg = 0;
    for (int i = start; i < end; i++)
    {
      avg = laser_data->ranges[i] + avg;
    }
    return avg/SWEEPING_RANGE;
  }

  float
  FollowWallLifeCycle::get_object_left(sensor_msgs::msg::LaserScan::SharedPtr laser_data){
    
    int start = (int)get_left_lecture(laser_data) - SWEEPING_RANGE/2;
    int end = (int)get_left_lecture(laser_data) + SWEEPING_RANGE/2;
    float avg = 0;
    for (int i = start; i < end; i++)
    {
      avg = laser_data->ranges[i] + avg;
    }
    return avg/SWEEPING_RANGE;
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
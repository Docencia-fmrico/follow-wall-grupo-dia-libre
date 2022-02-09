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
    is_first_turning_ = false;
            
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
    rclcpp::Time ts = now();
    RCLCPP_INFO(get_logger(), "Time now: %f",ts.seconds());
    RCLCPP_INFO(get_logger(), "Time started: %f",last_time_.seconds());
    RCLCPP_INFO(get_logger(), "Difference: %f",(ts - last_time_).seconds());

    if ((ts - last_time_).seconds() < TIME_TURNING)
    { 
      RCLCPP_INFO(get_logger(), "Turning");
      msg.linear.x = 0;
      msg.angular.z = direction * 0.5; // right - left +
      state_ = 0;
      return msg;
    }
    else
    {
      is_first_turning_ = false;
      state_ = 1;
      return msg;
    }
  }

  void  
  FollowWallLifeCycle::do_work() {
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }

    geometry_msgs::msg::Twist cmd;

    if (!state_)
    {
        if (!objectCenter && !is_first_turning_)
        {
          RCLCPP_INFO(get_logger(), "Forward, state 1");
          cmd.linear.x = 0.25;
          cmd.angular.z = 0;
        }
        else
        {
          if (!is_first_turning_)
          {
            last_time_ = now();
          }
          is_first_turning_ = true;
          cmd = turn(RIGHT);
        }
    }
    else
    {
      //RCLCPP_INFO(get_logger(), "STATE 2");
      if (!objectCenter && !objectLeft )
      {
        cmd.linear.x = 0;
        cmd.angular.z = 0.25;
      }
      else if(objectCenter)
      {
        cmd.linear.x = 0;
        cmd.angular.z = -0.25;
      }
      else
      {
        cmd.linear.x = 0.25;
        cmd.angular.z = 0;
      }
    }

    speed_pub_->publish(cmd);
    //RCLCPP_INFO(get_logger(), "State [%d]", state_);
      
  }

  bool
  FollowWallLifeCycle::get_object_right(sensor_msgs::msg::LaserScan::SharedPtr laser_data){
    int laser_third = laser_data->ranges.size()/3;
    int right_center = laser_third/2;
    int right_sweep_start = right_center - SWEEPING_RANGE/2;
    int right_sweep_end = right_center + SWEEPING_RANGE/2;
    float mean_right = 0;

    for (int i = right_sweep_start; i < right_sweep_end; i++)
    {
      mean_right = mean_right + laser_data->ranges[i];
    }
    mean_right = mean_right / SWEEPING_RANGE;
    //RCLCPP_INFO(get_logger(), "Object right: %d", mean_right < OBJECT_LIMIT);
  
    return mean_right < OBJECT_LIMIT;
  }

  bool
  FollowWallLifeCycle::get_object_center(sensor_msgs::msg::LaserScan::SharedPtr laser_data){
    int center_center = laser_data->ranges.size()/2;
    int center_sweep_start = center_center - SWEEPING_RANGE/2;
    int center_sweep_end = center_center + SWEEPING_RANGE/2;
    float mean_center = 0;

    for (int i = center_sweep_start; i < center_sweep_end; i++)
    {
      mean_center = mean_center + laser_data->ranges[i];
    }
    mean_center = mean_center / SWEEPING_RANGE;
    //RCLCPP_INFO(get_logger(), "Distance Center: %f", mean_center);
    //RCLCPP_INFO(get_logger(), "Object Center: %d", mean_center < OBJECT_LIMIT);
  

    return mean_center < OBJECT_LIMIT;
  }

  bool
  FollowWallLifeCycle::get_object_left(sensor_msgs::msg::LaserScan::SharedPtr laser_data){
    int laser_third = laser_data->ranges.size()/3;
    int left_center = laser_data->ranges.size() - laser_third/2;
    int left_sweep_start = left_center - SWEEPING_RANGE/2;
    int left_sweep_end = left_center + SWEEPING_RANGE/2;
    float mean_left = 0;

    for (int i = left_sweep_start; i < left_sweep_end; i++)
    {
      mean_left = mean_left + laser_data->ranges[i];
    }
    mean_left = mean_left / SWEEPING_RANGE;

    //RCLCPP_INFO(get_logger(), "Object Left: %d", mean_left < OBJECT_LIMIT);

    return mean_left < OBJECT_LIMIT;
  }
  
  void
  FollowWallLifeCycle::laser_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {    
    objectLeft = get_object_left(msg);
    objectCenter = get_object_center(msg);
    objectRight = get_object_right(msg);
  }

}
  
  // here we can define 3 zones in the msg->ranges : left center and right.
  // according to the value on those 3 zones, we can update a boolean for each:
  // objectLeft = true/false, objectright, objectcenter...
  // we can define those variables as private and update them in callback to use them in dowork.
#include "follow_wall/follow_wall.hpp"

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  FollowWallLifeCycle(): rclcpp_lifecycle::LifecycleNode("follow_wall_lifecycle")
  {
    //el topic de velocidad es /nav_vel y el tipo de mensaje es geometry_msgs/msg/Twist B)
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan_raw", 10, std::bind(&FollowWallLifeCycle::laser_cb, this, _1));
    speed_pub_ = create_publisher<geometry_msgs::msg::Twist>("/nav_vel", 10); //preguntar que poner aqui
  }

  CallbackReturnT FollowWallLifeCycle::on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());
            
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT FollowWallLifeCycle::on_activate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT FollowWallLifeCycle::on_deactivate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT FollowWallLifeCycle::on_cleanup(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT FollowWallLifeCycle::on_shutdown(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT FollowWallLifeCycle::on_error(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }

  void FollowWallLifeCycle::do_work() {
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }

    geometry_msgs::msg::Twist cmd;


    RCLCPP_INFO(get_logger(), "Node [%s] active", get_name());

    
    if(!objectCenter){
      cmd.linear.x = 0.25;
      cmd.angular.z = 0;
    }

    else if(objectCenter){
      cmd.linear.x = 0;
      cmd.angular.z = 0.25;
    }

    else if(objectLeft && !objectCenter)
    {
      cmd.linear.x = 0.25;
      cmd.angular.z = 0;
    }
    else if(objectLeft && objectCenter)
    {
      cmd.linear.x = 0;
      cmd.angular.z = 0.25;
    }

    // the idea will be to use the three boleans updatted by laser and work in the following way:
    // 1 go fowrard till we find a wall
    // 2 turn right on that wall.
    // 3 if there is an object on the left and not in the center, keep going forward
    // 4 if there is an object on the left AND on the center, turn right
    // repeat step 3.
  }

  void FollowWallLifeCycle::laser_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    int sz = msg->ranges.size();
    int f_left = (int)sz/3;
    int f_center = (int)2*sz/3;
    int f_right = sz;
    float limit = 0.35;

    float mean_left = 0;
    for (int i = 0; i < f_left; i++)
    {
      mean_left = mean_left + msg->ranges[i];
    }
    mean_left = mean_left / ((int)sz/3);
    if (mean_left < limit)
      objectLeft = false;
    else
      objectLeft = true;

    float mean_center = 0;
    for (int i = f_left; i < f_center; i++)
    {
      mean_center = mean_center + msg->ranges[i];
    }
    mean_center = mean_center / ((int)sz/3);

    if (mean_center < limit)
      objectCenter = false;
    else
      objectCenter = true;

    float mean_right = 0;
    for (int i = f_center; i < f_right; i++)
    {
      mean_right = mean_right + msg->ranges[i];
    }
    mean_right = mean_right / ((int)sz/3);


    if (mean_right < limit)
      objectRight = false;
    else
      objectRight = true;

    RCLCPP_INFO(this->get_logger(), "Media de izquierda: %f",
      mean_left);

    RCLCPP_INFO(this->get_logger(), "Media de centro: %f",
      mean_center);

    RCLCPP_INFO(this->get_logger(), "Media de derecha: %f",
      mean_right);

  }
  
  // here we can define 3 zones in the msg->ranges : left center and right.
  // according to the value on those 3 zones, we can update a boolean for each:
  // objectLeft = true/false, objectright, objectcenter...
  // we can define those variables as private and update them in callback to use them in dowork.


// Copyright 2022 Grupo Día Libre
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

FollowWallLifeCycle::FollowWallLifeCycle()
: rclcpp_lifecycle::LifecycleNode("follow_wall_lifecycle")
{
}

CallbackReturnT
FollowWallLifeCycle::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Node Configuring");

  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan_raw", 10, std::bind(&FollowWallLifeCycle::laser_cb, this, _1));

  speed_pub_ = create_publisher<geometry_msgs::msg::Twist>("/nav_vel", 10);
  state_ = 0;
  is_turning_ = 0;
  turn_to_ = 1;
  prev_mean_ = 0;
  min_dist_ = MAX_DIST_RANGE;
  count_it_trend_ = 0;
  prev_error_ = 0;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
FollowWallLifeCycle::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Node Activating");
  speed_pub_->on_activate();
  return CallbackReturnT::SUCCESS;
}
CallbackReturnT
FollowWallLifeCycle::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Node Deactivating");
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
FollowWallLifeCycle::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Node Cleaning Up");
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
FollowWallLifeCycle::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Node Shutting Down");
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
FollowWallLifeCycle::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Node Error: Shutting Down");
  return CallbackReturnT::SUCCESS;
}

geometry_msgs::msg::Twist
FollowWallLifeCycle::turn(int direction, float wvel)
{
  geometry_msgs::msg::Twist msg;

  if (direction == RIGHT) {
    msg.linear.x = 0;
    msg.angular.z = -wvel;
  } else {
    msg.linear.x = 0;
    msg.angular.z = wvel;
  }
  return msg;
}

bool
FollowWallLifeCycle::trend_algortihm(float dist)
{
  if (TREND_MAX_DIST > dist > TREND_MIN_DIST) {
    tend_mean_ += dist;
    tend_it_++;
  }

  if (tend_it_ == MAX_IT) {
    // RCLCPP_INFO(get_logger(), "Measure Left [%f]", distance_to_left_);
    // RCLCPP_INFO(get_logger(), "Min Left [%f]", min_dist_);
    // RCLCPP_INFO(get_logger(), "Prev Mean [%f]", prev_mean_);
    // RCLCPP_INFO(get_logger(), "Tend Mean [%f]", tend_mean_/MAX_IT);
    tend_it_ = 0;
    float trend = tend_mean_ / MAX_IT;
    if (!prev_mean_) {
      prev_mean_ = trend;
    } else {
      if (prev_mean_ > trend) {
        prev_mean_ = trend;
      } else {
        turn_to_ *= -1;
        count_it_trend_++;
        if (dist <= min_dist_ + DIST_VARIATION || count_it_trend_ == MAX_RECALCULATIONS) {
          // RCLCPP_INFO(get_logger(), "Finish");
          count_it_trend_ = 0;
          prev_mean_ = 0;
          min_dist_ = MAX_DIST_RANGE;
          return true;
        }
      }
    }
    if (min_dist_ > trend && trend > FLOAT_ZERO) {
      min_dist_ = trend;
    }
    tend_mean_ = 0;
  }

  return false;
}

void
FollowWallLifeCycle::do_work()
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  geometry_msgs::msg::Twist cmd;
  // RCLCPP_INFO(get_logger(), "Measure Center [%f]", distance_to_center_);

  if (distance_to_center_ == 0 || distance_to_left_ == 0) {
    return;  // at the beggining laser seems empty so avoid that
  }

  if (distance_to_center_ < OBJECT_LIMIT || is_turning_ == 1) {
    if (is_turning_ != 1) {
      turn_to_ = 1;
    }

    is_turning_ = 1;
    cmd = turn(turn_to_ * RIGHT, 0.2);

    count_it_rot_++;
    if (count_it_rot_ >= MAX_ROTATIONS) {
      if (trend_algortihm(distance_to_left_)) {
        count_it_rot_ = 0;
        is_turning_ = 0;
        state_ = 1;
      }
    }
  } else {
    if (state_) {
      cmd.linear.x = LINEAR_SPEED - (distance_to_left_ - OBJECT_LIMIT) / 2;
      cmd.angular.z = (distance_to_left_ - OBJECT_LIMIT - 0.1) +
        (distance_to_left_ - OBJECT_LIMIT - 0.1 - prev_error_) * ANGULAR_KD;
      prev_error_ = distance_to_left_ - OBJECT_LIMIT;

      if (distance_to_left_ - OBJECT_LIMIT > 1) {
        turning_left_ = 1;
      } else {
        turning_left_ = 0;
      }
    } else {
      cmd.linear.x = LINEAR_SPEED;
      cmd.angular.z = 0;
    }
  }

  speed_pub_->publish(cmd);
  // RCLCPP_INFO(get_logger(), "State [%d]", state_);
}

float
FollowWallLifeCycle::get_left_lecture(sensor_msgs::msg::LaserScan::SharedPtr laser_data)
{
  return laser_data->ranges.size() / 2 + (LEFT_DETECTION_ANGLE) / laser_data->angle_increment;
}

float
FollowWallLifeCycle::get_object_center(sensor_msgs::msg::LaserScan::SharedPtr laser_data)
{
  int start = laser_data->ranges.size() / 2 - SWEEPING_RANGE / 2;
  int end = laser_data->ranges.size() / 2 + SWEEPING_RANGE / 2;
  float avg = 0;
  for (int i = start; i < end; i++) {
    avg = laser_data->ranges[i] + avg;
  }
  return avg / SWEEPING_RANGE;
}

float
FollowWallLifeCycle::get_object_left(sensor_msgs::msg::LaserScan::SharedPtr laser_data)
{
  int start = static_cast<int>(get_left_lecture(laser_data) - SWEEPING_RANGE / 2);
  int end = static_cast<int>(get_left_lecture(laser_data) + SWEEPING_RANGE / 2);
  float avg = 0;
  for (int i = start; i < end; i++) {
    avg = laser_data->ranges[i] + avg;
  }
  return avg / SWEEPING_RANGE;
}

void
FollowWallLifeCycle::laser_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  distance_to_left_ = get_object_left(msg);
  distance_to_center_ = get_object_center(msg);
}

}  // namespace follow_wall

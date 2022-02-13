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

#include <memory>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "gtest/gtest.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "follow_wall/follow_wall.hpp"

class FollowWallLifeCycleTest : public follow_wall::FollowWallLifeCycle
{
private:
  float test_left_;
  float test_center_;

public:
  void
  set_search()
  {
    state_ = 1;
  }

  void
  set_test_values(float left_value, float center_value)
  {
    distance_to_left_ = left_value;
    distance_to_center_ = center_value;
  }
  int
  get_turning_right()
  {
    return is_turning_;
  }
  int
  get_turning_left()
  {
    return turning_left_;
  }
};

TEST(test_node, right_turn_test)
{
  auto node = std::make_shared<FollowWallLifeCycleTest>();

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  node->set_search();

  node->set_test_values(0.4, 0.4);

  node->do_work();

  ASSERT_EQ(node->get_turning_right(), 1);
}

TEST(test_node, straight_test)
{
  auto node = std::make_shared<FollowWallLifeCycleTest>();

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  node->set_search();

  node->set_test_values(0.4, 0.8);

  node->do_work();

  ASSERT_EQ(node->get_turning_right(), 0);
  ASSERT_EQ(node->get_turning_left(), 0);
}

TEST(test_node, left_turn_test)
{
  auto node = std::make_shared<FollowWallLifeCycleTest>();

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  node->set_search();

  node->set_test_values(12, 12);

  node->do_work();

  ASSERT_EQ(node->get_turning_left(), 1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}

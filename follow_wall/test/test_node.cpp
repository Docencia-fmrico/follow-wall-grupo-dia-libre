// #include hay que incluir la cabecera de nuestro nodo para poder usar las clases y funciones


// preguntas, pasarcosas por parametrs, para el ts
// lo de partir el laser en 3



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
  RCLCPP_INFO( get_logger()," state:  \n" + std::to_string(state_) + "\n");
}

void
get_test_values()
{
  RCLCPP_INFO( get_logger(),"left:  \n" + std::to_string(distance_to_left_) + "\n");
  RCLCPP_INFO( get_logger(),"right: \n" + std::to_string(distance_to_left_) + "\n");
}

void
set_test_values(float left_value, float center_value)
{
  distance_to_left_ = left_value;
  distance_to_center_ = center_value;

  RCLCPP_INFO( get_logger(),"left:  \n" + std::to_string(distance_to_left_) + "\n");
  RCLCPP_INFO( get_logger(),"right: \n" + std::to_string(distance_to_left_) + "\n");
}

void
laser_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  distance_to_left_ = test_left_;
  distance_to_center_ = test_center_;

}

int
get_turning()
{

  RCLCPP_INFO( get_logger(),"\n" + std::to_string(is_turning_) + "\n");
  return is_turning_;
}
  
};


TEST(test_node, test_node)
{
  auto node = std::make_shared<FollowWallLifeCycleTest>();

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  node->set_search();

  node->set_test_values(0.4, 0.4);

  node->do_work();

  node->get_test_values();

  ASSERT_EQ(node->get_turning(), 1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}


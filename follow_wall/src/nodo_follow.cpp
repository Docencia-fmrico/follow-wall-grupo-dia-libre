#include <memory>

#include "follow_wall_node.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


int main(int argc, char * argv[])
{
  auto node = std::make_shared<FollowWallLifeCycle>();

  rclcpp::init(argc, argv);  

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->do_work();

    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }
  

  rclcpp::shutdown();

  return 0;
}
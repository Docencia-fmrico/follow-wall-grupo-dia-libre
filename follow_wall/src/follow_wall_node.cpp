#include "follow_wall/follow_wall.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<follow_wall::FollowWallLifeCycle>();

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->do_work();

    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }
  
  rclcpp::shutdown();

  return 0;
}
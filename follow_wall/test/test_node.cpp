
#include <memory>
#include "lifecycle_msgs/msg/state.hpp"
#include "gtest/gtest.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "test_ci/TestNode.hpp"
#include "follow_wall/follow_wall.hpp"

//#include hay que incluir la cabecera de nuestro nodo para poder usar las clases y funciones


//preguntas, pasarcosas por parametrs, para el ts
//lo de partir el laser en 3
//
TEST( test_node, test_laser_positions)
{

  auto node = std::make_shared<FollowWallLifeCycle>();

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);


  sensor_msgs::msg::LaserScan laser;

  //estableceemos la cabecera del laser con valores adecuados
  laser.header.stamp = node->now(); 
  laser.angle_min = -M_PI;
  laser.angle_max = M_PI;

  laser.angle_increment = 2.0 * M_PI / 3.0f;

  //establecemos que el rango de barrido se divida en tres sectores
  laser.ranges = std::vector<float>( 3, 15.0f);

  //en el primer campo rellenamos con un valor para simular una detección de laser
  laser.ranges[0] = 0.3f;

  node->mean_left = laser.ranges[0];
  node->mean_center = laser.ranges[1];
  node->mean_right = laser.ranges[2];


  auto vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/nav_vel", 10, std::bind(NULL, this, _1));


  ASSERT_LT(0, vel_pub.cmd.angular.z);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
3
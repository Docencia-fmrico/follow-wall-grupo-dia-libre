#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

class FollowWallLifeCycle : public rclcpp_lifecycle::LifecycleNode
{
public:
  FollowWallLifeCycle()
  : rclcpp_lifecycle::LifecycleNode("follow_wall_lifecycle")
  {
    //el topic de velocidad es /nav_vel y el tipo de mensaje es geometry_msgs/msg/Twist B)
    laser_sub_ = create_subscription<std_msgs::msg::String>(
      "scan_raw", 10, std::bind(&MyNodeSubscriber::callback, this, _1));
    speed_pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
  }

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());
   
    speed_ = get_parameter("speed").get_value<double>();
   
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }

    CallbackReturnT on_error(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }
  
  void do_work() {
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }

    RCLCPP_INFO(get_logger(), "Node [%s] active", get_name());
  }


private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr laser_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speed_pub_;
};


void callback(const std_msgs::msg::String::SharedPtr msg)
{
  //el callback del nodo añadidio no hace nada, podriamos ponerle algun mensjae de depuracion
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LifeCycleNodeExample>();


  rclcpp::init(argc, argv);

  //eh hecho un nodo aparte que se suscriba pq en principio los lifecycle nodes polludos estos no implementan subscripcion
  
  rclcpp::spin(sub_);

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->do_work();

    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }
  

  rclcpp::shutdown();

  return 0;
}
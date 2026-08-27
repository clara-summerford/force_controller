#include "force_controller/hybrid_controller_action_server.hpp"
#include "rclcpp/rclcpp.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create the controller action server object (which IS a ROS node)
  auto node = std::make_shared<hybrid_multidim_control::HybridControllerActionServer>();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
  
}
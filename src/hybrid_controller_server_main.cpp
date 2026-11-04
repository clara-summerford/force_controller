#include "force_controller/hybrid_controller_action_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include <thread>


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create ROS node to pass into hybrid controller action server
  auto node = std::make_shared<rclcpp::Node>("hybrid_controller_action_server");

  // Create the controller action server object 
  hybrid_multidim_control::HybridControllerActionServer server = 
    hybrid_multidim_control::HybridControllerActionServer(node);
  

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
  
}
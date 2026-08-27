#include "force_controller/force_controller.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // Create the ForceController object.
  auto force_controller =
      std::make_shared<twist_force_controller::ForceController>();

  // Initialize the force controller.
  if (!force_controller->initialize()) {
    RCLCPP_ERROR(
        rclcpp::get_logger("main"),
        "Failed to initialize force controller node. Shutting down node.");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  // Spin the node
  rclcpp::spin(force_controller);

  // Shutdown the node
  rclcpp::shutdown();

  return 0;
}
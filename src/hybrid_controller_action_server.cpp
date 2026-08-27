#include "force_controller/hybrid_controller_action_server.hpp"

namespace hybrid_multidim_control
{

HybridControllerActionServer::HybridControllerActionServer(
    const rclcpp::NodeOptions & options)
    : Node("hybrid_controller_action_server", options)
{
    using namespace std::placeholders;

    // hybrid_controller is REAL name of action as seen by ROS
    action_server_ = rclcpp_action::create_server<HybridController>(
        this,
        "hybrid_controller",
        std::bind(&HybridControllerActionServer::handle_goal, this, _1, _2),
        std::bind(&HybridControllerActionServer::handle_cancel, this, _1),
        std::bind(&HybridControllerActionServer::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse HybridControllerActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const HybridController::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with max force: %.2f", goal->max_force);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse HybridControllerActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleHybridController> goal_handle)
{ 
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void HybridControllerActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleHybridController> goal_handle)
{
    using namespace std::placeholders;
    std::thread{std::bind(&HybridControllerActionServer::execute, this, _1), goal_handle}.detach();
}


void HybridControllerActionServer::execute(
    const std::shared_ptr<GoalHandleHybridController> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<HybridController::Feedback>();
    auto result = std::make_shared<HybridController::Result>();


    RCLCPP_INFO(this->get_logger(), "Executing goal yay!!!!");

    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
    }

    for (int i = 0; i < 2; ++i) {
        feedback->current_wrench.force.z = 5;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback %d", i);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    
    // Publish feedback
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Publish feedback");


    // Check if goal is done
    if (rclcpp::ok()) {
      result->message = "beep boop";
      result->success = true;
      RCLCPP_INFO(this->get_logger(), "Result before succeed: %d", result->success);

      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      RCLCPP_INFO(this->get_logger(), "Result: %d", result->success);

    }
}

} // namespace hybrid_multidim_control

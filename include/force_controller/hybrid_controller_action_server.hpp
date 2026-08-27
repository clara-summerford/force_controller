#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "force_controller/action/hybrid_controller.hpp"


namespace hybrid_multidim_control
{

class HybridControllerActionServer : public rclcpp::Node
{
public:
  using HybridController = force_controller::action::HybridController;
  using GoalHandleHybridController = rclcpp_action::ServerGoalHandle<HybridController>;

  /**
   * @brief Constructor for the hybrid force motion controller action server.
   * @param options --- TO DO ---
   * @details --- TO DO ---
   */

  explicit HybridControllerActionServer(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<HybridController>::SharedPtr action_server_;

  /**
   * @brief Function to handle incoming goals from the action request from the client. 
   * @return Returns response to action client through the action once complete. 
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const HybridController::Goal> goal);

  /**
   * @brief ----
   * @return ---
   */  
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleHybridController> goal_handle);

  /**
  * @brief ---
  */
  void handle_accepted(const std::shared_ptr<GoalHandleHybridController> goal_handle);

  /**
  * @brief ---
  */
  void execute(const std::shared_ptr<GoalHandleHybridController> goal_handle);


}; // class HybridControllerActionServer

} // namespace hybrid_multidim_control
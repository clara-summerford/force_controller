#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "force_controller/action/hybrid_controller.hpp"
#include "force_controller/force_controller.hpp"
#include "force_controller/utils/param_handler.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

namespace hybrid_multidim_control {

class HybridControllerActionServer 
{
public:
  using HybridController = force_controller::action::HybridController;
  using GoalHandleHybridController = rclcpp_action::ServerGoalHandle<HybridController>;

  /**
   * @brief Constructor for the hybrid force motion controller action server.
   * @param node Standard ROS node.
   */

  HybridControllerActionServer(const rclcpp::Node::SharedPtr& node);


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
   * @brief Function to run when the incoking goal request is cancelled.
   * @return ROS Action cancel response. 
   */  
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleHybridController> goal_handle);

  /**
  * @brief Accept incoming action goal. 
  */
  void handle_accepted(const std::shared_ptr<GoalHandleHybridController> goal_handle);

  /**
  * @brief Execute code in response to incoming action goal. 
  */
  void execute(const std::shared_ptr<GoalHandleHybridController> goal_handle);

  // Helper functions

    inline std::vector<std::string> splitString(const std::string &input) {
    std::stringstream ss(input);
    std::vector<std::string> results;
    std::string token;

    while (std::getline(ss, token, ',')) {
      results.push_back(token);
    }

    return results;
  }

  /**
   * @brief Converts action goal components into a struct to hold all goal params. 
   * 
   * @param goal 
   * @return HybridGoalParams 
   */
  HybridGoalParams extractGoalParams(  
    const std::shared_ptr<const HybridController::Goal>& goal);

  /**
   * @brief Combines all three param structs into one struct. 
   * 
   * @return HybridForceExecParams 
   */
  HybridForceExecParams buildExecParams(
     const twist_force_controller::ForceControllerParams& force,
     const HybridMotionParams& hybrid,
     const HybridGoalParams& goal);

  /**
   * @brief Load the Params object from the force controller, hybrid controller
   * and action goals. Flattens all params into an easy to use param struct.
   * @param goal Action client goal containing params relevant to hybrid force
   * motion planning. 
   * @return Bool if params were gathered successfully. 
   */
  bool getParams(std::shared_ptr<const HybridController::Goal> goal);

  /**
   * @brief Publishes the twist message to the robot's end effector frame.
   * @details This function publishes the twist message to the robot's end
   * effector frame at a specified rate. The twist message is published for a
   * specified duration or a specified distance, after which the motion stops.
   * @post The Behavior publishes end-effector twists for simple Cartesian
   * motion primitives in a given dimension.
   */
  void publishTwist();

  /**
   * @brief Stops the motion after the desired time has elapsed.
   */
  void stopMotion();

  /**
   * @brief Sets the twist value for the desired dimension.
   * @param twist The twist message to be set.
   */
  void setTwistValue(geometry_msgs::msg::TwistStamped &twist);

  /**
   * @brief Sets the wrench value for the desired variable wrenches.
   * @param wrench The wrench message to be set.
   */
  void setWrenchValue(geometry_msgs::msg::WrenchStamped &wrench);

  /**
   * @brief Checks if the dimension is valid.
   * @param dimension The dimension to be checked.
   * @return bool
   */
  bool isValidDimension(const std::string &dimension);

  /**
   * @brief Checks if a value passed is valid.
   * @param value The twist value to be checked.
   * @return bool
   */
  bool isValidValue(double value);

  /**
   * @brief Checks the current pose of the end-effector to decide whether to
   * decide whether to continue publishing or to stop. If the end-effector has
   * moved the desired distance, the motion stops.
   * @post The Behavior stops the motion if the end-effector has moved the
   * desired distance.
   */
  void checkEndEffectorPose();

  /**
   * @brief Updates the initial pose of the end effector.
   */
  bool updateInitialPose();

  /**
   * @brief Converts radians to degrees.
   * @param radians The angle in radians to be converted.
   * @return double The angle in degrees.
   */
  double toDegrees(double radians) { return radians * 180.0 / M_PI; }

  /**
   * @brief Checks if the motion is linear.
   */
  bool isLinear();

  /**
   * @brief Checks if the motion is a pre-defined shape.
   */
  bool isShape();

  /**
   * @brief Checks if the motion is an RPY angle.
   */
  bool isAngular();

  /**
   * @brief Initializes the robot model loader.
   * @return BT::NodeStatus
   */
  bool initializeRobotModelLoader();

  /**
   * @brief Callback function to update the joint state data.
   * @note This is needed to allow for the proper update of the robot state so
   * that the end effector pose can be accurately computed.
   * @param msg The joint state message received.
   */
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  /**
   * @brief Callback function to update the admittance controller data.
   * @note This is needed to allow for the proper update of the robot state so
   * that the end effector pose can be accurately computed.
   * @param msg The twist message received.
   */
  void admittanceControllerCallback(
      const geometry_msgs::msg::TwistStamped::SharedPtr msg);
      
  /**
   * @brief Adds two twist messages.
   * @param twist1 The first twist message.
   * @param twist2 The second twist message.
   * @return geometry_msgs::msg::Twist The sum of the two twist messages.
   */
  geometry_msgs::msg::Twist addTwists(const geometry_msgs::msg::Twist &twist1,
                                      const geometry_msgs::msg::Twist &twist2);


  // Private member variables 
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;

  std::chrono::time_point<std::chrono::steady_clock> start_time_;
  moveit::core::RobotStatePtr robot_state_;
  const moveit::core::JointModelGroup *joint_model_group_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  sensor_msgs::msg::JointState::SharedPtr current_joint_state_{nullptr};
  
  geometry_msgs::msg::Pose initial_pose_;
  bool initial_pose_set_{false};
  bool distance_reached_{false};

  double angular_twist_val_{0.2}; // RPY angular twist value
  double t_; // time difference for nonlinear paths
  rclcpp::Time path_start_time_; // start time for nonlinear paths

  rclcpp::Node::SharedPtr node_;

  bool force_controller_initialized_ = false;
  std::shared_ptr<twist_force_controller::ForceController> force_controller_;
  twist_force_controller::ForceControllerParams force_controller_params_;
  HybridMotionParams hybrid_motion_params_;
  HybridGoalParams hybrid_goal_params_;
  HybridForceExecParams params_;

}; // class HybridControllerActionServer

} // namespace hybrid_multidim_control
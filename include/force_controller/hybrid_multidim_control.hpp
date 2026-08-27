#pragma once

#include <behaviortree_cpp/action_node.h>

#include "force_controller/force_controller.hpp"
#include "glovebox_bt/utils/param_handler.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/node.hpp> // for getNode()
#include <tl/expected.hpp> //testing


namespace hybrid_multidim_control
{
/**
 * @brief TODO(...)
 */


const double kDefaultMaxTwist = 0.05; // Default maximum twist value

struct MotionPrimitiveConfig {
  /**
   * @brief This struct contains the configuration parameters for the Cartesian
   * Motion Primitive behavior. The parameters are set in the behavior tree's
   * XML file.
   * @details The parameters are used to set the time length for which the
   * Cartesian motion primitive is executed, the twist value, the maximum twist
   * value, the distance delta, the dimension in which the motion is executed,
   * the frame in which the twist is published, the topic on which the twist
   * is published, and the group name of the joint model group.
   */
  double publish_time_length;
  double twist_val;
  double max_twist;
  double distance_delta;
  std::string dimension;
  std::string twist_pub_frame;
  std::string pub_topic;
  std::string group_name;
};


class HybridMultidimControl : public BT::StatefulActionNode
{
public:
  /**
   * @brief Constructor for the hybrid_multidim_control behavior.
   * @param name The name of a particular instance of this Behavior. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @param shared_resources A shared_ptr to a BehaviorContext that is shared among all SharedResourcesNode Behaviors in the behavior tree. This BehaviorContext is owned by the Studio Agent's ObjectiveServerNode.
   * @param config This contains runtime configuration info for this Behavior, such as the mapping between the Behavior's data ports on the behavior tree's blackboard. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @details An important limitation is that the members of the base Behavior class are not instantiated until after the initialize() function is called, so these classes should not be used within the constructor.
   */
  // HybridMultidimControl(
  //   const std::string& name, const BT::NodeConfiguration& config);

  // quick testing for non-moveit pro nodes
  HybridMultidimControl(
        const std::string& name, 
        const BT::NodeConfig& config,
        std::weak_ptr<rclcpp::Node> node_ptr = {},
        tf2_ros::Buffer::SharedPtr tf_buffer = nullptr
        );


  ~HybridMultidimControl() override;

  /**
   * @brief Implementation of the required providedPorts() function for the hybrid_multidim_control Behavior.
   * @details The BehaviorTree.CPP library requires that Behaviors must implement a static function named providedPorts() which defines their input and output ports. If the Behavior does not use any ports, this function must return an empty BT::PortsList.
   * This function returns a list of ports with their names and port info, which is used internally by the behavior tree.
   * @return hybrid_multidim_control does not use expose any ports, so this function returns an empty list.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of the metadata() function for displaying metadata, such as Behavior description and
   * subcategory, in the MoveIt Studio Developer Tool.
   * @return A BT::KeyValueVector containing the Behavior metadata.
   */
  static BT::KeyValueVector metadata();

  /**
   * @brief Implementation of onStart(). Runs when the Behavior is ticked for the first time.
   * @return Always returns BT::NodeStatus::RUNNING, since the success of Behavior's initialization is checked in @ref
   * onRunning().
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief Implementation of onRunning(). Checks the status of the Behavior when it is ticked after it starts running.
   * @return TODO(...)
   */
  BT::NodeStatus onRunning() override;

  void onHalted() override;

protected:
  /**
   * @brief Retrieves node from SharedPtr
   * @return ROS node.
   */
  rclcpp::Node::SharedPtr getNode() const {
      rclcpp::Node::SharedPtr node = node_ptr_.lock();
      if (!node) {
          // RCLCPP_ERROR(get_logger(), "The ROS node has gone out of scope. You are operating with a null pointer!");
          std::cout << "The ROS node has gone out of scope. You are operating with a null pointer!" << std::endl; // FIX THIS
      }
      return node;
    }

  // rclcpp::Logger get_logger() const {return logger_;}
  
private:
  /**
   * @brief Callback function for the force controller's wrench topic.
   *
   * @param msg The wrench message from the force controller.
   */
  void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

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
   * @brief Grabs the parameters from the blackboard and stores them in the
   * class variables.
   * @return A tl::expected<bool, std::string> containing a bool if the
   * parameter was successfully retrieved, or an error message if the parameter
   * was not successfully retrieved.
   */
  tl::expected<bool, std::string> getParams();

  /**
   * @brief Gets the value of a parameter from the blackboard
   * @details This function is used to get the value of a parameter from the
   * blackboard. It is a template function that can be used to get the value of
   * any type of parameter from the blackboard.
   * @param param_name The name of the parameter to get.
   * @param storage_value The variable to store the value of the parameter
   * @return A tl::expected<bool, std::string> containing a bool if the
   * parameter was successfully retrieved, or an error message if the parameter
   * was not successfully retrieved.
   */
  template <typename T>
  tl::expected<bool, std::string> getValue(const std::string &param_name,
                                           T &storage_value);

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



  std::weak_ptr<rclcpp::Node> node_ptr_; // for getNode()
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::optional<tf2_ros::TransformListener> tf_listener_;
  // rclcpp::Logger logger_; // for getLogger()


  rclcpp::Node::SharedPtr node_; // implemenataion TODO
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;

  std::chrono::time_point<std::chrono::steady_clock> start_time_;
  moveit::core::RobotStatePtr robot_state_;
  const moveit::core::JointModelGroup *joint_model_group_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      admittance_controller_sub_;
  sensor_msgs::msg::JointState::SharedPtr current_joint_state_{nullptr};
  
  geometry_msgs::msg::Pose initial_pose_;
  bool initial_pose_set_{false};
  bool distance_reached_{false};

  double angular_twist_val_{0.2}; // RPY angular twist value
  double t_; // time difference for nonlinear paths
  rclcpp::Time path_start_time_; // start time for nonlinear paths

  std::shared_ptr<twist_force_controller::ForceControllerParams>
      force_controller_params_;
  std::shared_ptr<twist_force_controller::ForceController> force_controller_;

  // The configuration parameters for the Hybrid Force-Motion behavior
  std::shared_ptr<MotionPrimitiveConfig> hybrid_motion_primitive_config_;

  bool add_admittance_control_{false};
  geometry_msgs::msg::TwistStamped::SharedPtr latest_admittance_twist_;


};
}  // namespace hybrid_multidim_control

#include <glovebox_bt/behaviors/hybrid_multidim_control.hpp>
#include "glovebox_bt/utils/utils.hpp"
#include <algorithm> // For std::clamp
// #include <moveit_studio_behavior_interface/check_for_error.hpp> // removing this should flag some error???
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tl/expected.hpp> //testing



using namespace hybrid_multidim_utils;
namespace hybrid_multidim_control
{

HybridMultidimControl::HybridMultidimControl(
    const std::string& name, 
    const BT::NodeConfiguration& config,
    //testing
    std::weak_ptr<rclcpp::Node> node_ptr,
    tf2_ros::Buffer::SharedPtr tf_buffer)

  : BT::StatefulActionNode(name, config),

  tf_buffer_(tf_buffer),
  node_ptr_(node_ptr)
  
{ // Retrieve ROS node from blackboard TO DOOOOOOOOOO
  config.blackboard->get("node", node_);
}


// Create a rclcpp Logger to use when printing messages to the console.
const rclcpp::Logger kLogger =
    rclcpp::get_logger("HybridMultidimControl");


BT::PortsList HybridMultidimControl::providedPorts()
{
  return {
      BT::InputPort<std::vector<double>>("wrench_setpoint"),
      BT::InputPort<double>("kp"),
      BT::InputPort<double>("ki"),
      BT::InputPort<double>("kd"),
      BT::InputPort<std::vector<double>>("damping"),
      BT::InputPort<std::vector<double>>("stiffness"),
      BT::InputPort<double>("ee_weight"),
      BT::InputPort<std::string>("ft_sensor_frame"),
      BT::InputPort<std::string>("pub_topic"),
      BT::InputPort<std::string>("wrench_sub_topic"),
      BT::InputPort<double>("max_force"),
      BT::InputPort<double>("max_twist"),
      BT::InputPort<std::string>("filter_order"),
      BT::InputPort<double>("motion_publish_time_length"),
      BT::InputPort<double>("motion_twist_val"),
      BT::InputPort<double>("motion_distance_delta"),
      BT::InputPort<std::string>("motion_dimension"),
      BT::InputPort<std::string>("group_name"),
      BT::InputPort<bool>("add_admittance_control"),
  };
}

// BT::KeyValueVector HybridMultidimControl::metadata()
// {
//   // TODO(...)
//   return { {"description", "New version of multidim hybrid controller"} };
// }

template <typename T>
tl::expected<bool, std::string>
HybridMultidimControl::getValue(const std::string &param_name,
                                   T &storage_value) {
  auto value = getInput<T>(param_name);

  // TO DOOOOOO
  // Check that all required input data ports were set 
  // if (const auto error = moveit_studio::behaviors::maybe_error(value)) {
  //   return tl::make_unexpected("Failed to retrieve parameter '" + param_name +
  //                              "'. Error: " + error.value());
  // }

  storage_value = value.value();
  return true;
}

tl::expected<bool, std::string> HybridMultidimControl::getParams() {
  // Get the required parameters
  hybrid_motion_primitive_config_ = std::make_shared<MotionPrimitiveConfig>();

  force_controller_params_ =
      std::make_shared<twist_force_controller::ForceControllerParams>(
          twist_force_controller::getDefaultControllerParams());

  // Create a vector of param names and pointers to the double values
  std::vector<std::string> param_names = {//"force_setpoint",
                                          "kp",
                                          "ki",
                                          "kd",
                                          "ee_weight",
                                          "max_force",
                                          "max_twist",
                                          "motion_publish_time_length",
                                          "motion_twist_val",
                                          "motion_distance_delta"};

  std::vector<double *> param_values = {
      // &force_controller_params_->force_setpoint,
      &force_controller_params_->kp,
      &force_controller_params_->ki,
      &force_controller_params_->kd,
      &force_controller_params_->ee_weight,
      &force_controller_params_->max_force,
      &hybrid_motion_primitive_config_->max_twist,
      &hybrid_motion_primitive_config_->publish_time_length, /*motion param*/
      &hybrid_motion_primitive_config_->twist_val, /*motion_twist_val*/
      &hybrid_motion_primitive_config_->distance_delta /*motion_distance_delta*/};

  // Iterate through the parameter names and values, and retrieve the values
  for (size_t i = 0; i < param_names.size(); i++) {
    auto param_result = getValue(param_names.at(i), *param_values.at(i));
    if (!param_result) {
      // Return an unexpected value with the error message

      return tl::make_unexpected(param_result.error());
    }
  }

  // Handle special double vector params
  // double damping, stiffness;
  std::vector<double> wrench_setpoint, damping, stiffness;

  auto wrench_setpoint_result = getValue("wrench_setpoint", wrench_setpoint);
  auto damping_result = getValue("damping", damping);
  auto stiffness_result = getValue("stiffness", stiffness);

  if (!damping_result) {
    return tl::make_unexpected("Failed to retrieve parameter: damping");
  }

  // force_controller_params_->damping_1D.push_back(damping);
  force_controller_params_->damping_6D = damping; 

  if (!stiffness_result) {
    return tl::make_unexpected("Failed to retrieve parameter: stiffness");
  }
  // force_controller_params_->stiffness_1D.push_back(stiffness);
  force_controller_params_->stiffness_6D = stiffness;

  if (!wrench_setpoint_result) {
    return tl::make_unexpected("Failed to retrieve parameter: wrench_setpoint");
  }
  force_controller_params_->control_wrench = true; // may or may not need???
  force_controller_params_->wrench_setpoint = wrench_setpoint;

  // Create a vector of parameter names pointers to the string values
  std::vector<std::string> string_param_names = {
      "ft_sensor_frame",  "pub_topic", 
      "wrench_sub_topic",  "motion_dimension", "group_name"};
  std::vector<std::string *> string_param_values = {
      // &force_controller_params_->dimension_str, /*force_dimension_str*/
      &force_controller_params_->ft_sensor_frame,
      // &force_controller_params_->gravity_frame,
      &force_controller_params_->pub_topic,
      &force_controller_params_->wrench_sub_topic,
      &hybrid_motion_primitive_config_->dimension, /*motion_dimension*/
      &hybrid_motion_primitive_config_->group_name};

  for (size_t i = 0; i < string_param_names.size(); i++) {
    auto param_result =
        getValue(string_param_names.at(i), *string_param_values.at(i));
    if (!param_result) {
      return tl::make_unexpected(param_result.error());
    }
  }

  std::string filter_order;
  auto filter_order_result = getValue("filter_order", filter_order);
  if (!getValue("filter_order", filter_order))
    return tl::make_unexpected("Failed to retrieve parameter: filter_order");

  if (!filter_order_result) {
    return tl::make_unexpected("Failed to retrieve parameter: filter_order");
  }

  // Split the filter_order string into as a vector of filter strings
  std::vector<std::string> filter_order_vec = splitString(filter_order);

  for (const std::string &word : filter_order_vec) {
    // Print the filters in the filter_order to the console
    // RCLCPP_WARN(kLogger, "Filter type: " << word); // this logger structure doesn't work!!!
    // Check if the filter_order parameter was successfully retrieved
    if (word != "EMA" && word != "Butterworth" && word != "Kalman") {
      // RCLCPP_ERROR(kLogger, "Invalid filter type: " << word);
      return tl::make_unexpected(
          "Invalid filter type: " + word +
          ". Valid filter types are: EMA, Butterworth, Kalman");
    }

    force_controller_params_->filter_params.filter_order.push_back(
        filter_utils::stringToFilterType(word));
  }

  // Get the add_admittance_control parameter
  auto add_admittance_control_result =
      getValue("add_admittance_control", add_admittance_control_);
  if (!add_admittance_control_result) {
    return tl::make_unexpected(
        "Failed to retrieve parameter: add_admittance_control");
  }

  // Set the remaining controller parameters
  force_controller_params_->twist_pub_frame = "end_effector_link"; // testing, should prob be grasp_link
  force_controller_params_->gravity_frame = "base_link";
  force_controller_params_->add_restoring_force_term = false;
  force_controller_params_->get_controller_twist = true;
  force_controller_params_->use_PID = true; 
  // force_controller_params_->damping_1D = {10.0};
  force_controller_params_->max_twist = hybrid_motion_primitive_config_->max_twist; 

  return true;
}

bool HybridMultidimControl::isValidDimension(const std::string &dimension) {
  // List of valid dimensions
  const std::vector<std::string> validDimensions = {
      "x", "X", "y", "Y", "z", "Z", "roll", "pitch", "yaw", 
      "circle", "Circle", "square", "Square", "s"};
  return std::find(validDimensions.begin(), validDimensions.end(), dimension) !=
         validDimensions.end();
}

bool HybridMultidimControl::isValidValue(double value) {
  // Check if the passed values are valid numbers
  if (std::isnan(value) || std::isinf(value)) {
    return false;
  }
  return true;
}

void HybridMultidimControl::wrenchCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  // Call the updateFTCB method of the ForceController instance
  force_controller_->triggerUpdateFTCB(msg);
}

// testing variable wrench to publish
void HybridMultidimControl::setWrenchValue(
    geometry_msgs::msg::WrenchStamped &wrench) {

  std::string dim = hybrid_motion_primitive_config_->dimension;
  std::transform(dim.begin(), dim.end(), dim.begin(), ::tolower);
  // const double max_force = force_controller_params_->max_force;
  const double twist_val = hybrid_motion_primitive_config_->twist_val;
  const double distance_delta = hybrid_motion_primitive_config_->distance_delta;
  const std::vector<double> wrench_set = force_controller_params_->wrench_setpoint;


  // Calculating time interval for twist calculations for non-linear paths
  // const double t_int = distance_delta/twist_val;
  const double w = twist_val/distance_delta;

  if (dim == "circle"){
    wrench.wrench.force.x = -wrench_set[0]*sin(w*t_); // negative to oppose direction of motion
    wrench.wrench.force.y = wrench_set[1]*cos(w*t_);
    wrench.wrench.force.z = wrench_set[2];
  } else {
    hybrid_multidim_utils::toWrenchMsg(wrench, wrench_set);
  }
}

void HybridMultidimControl::setTwistValue(
    geometry_msgs::msg::TwistStamped &twist) {
  const double twist_val = hybrid_motion_primitive_config_->twist_val;
  const double distance_delta = hybrid_motion_primitive_config_->distance_delta;

  // Convert the input dimension to lowercase for case-insensitive comparison
  std::string dim = hybrid_motion_primitive_config_->dimension;
  std::transform(dim.begin(), dim.end(), dim.begin(), ::tolower);

  // Calculating time interval for twist calculations for non-linear paths
  const double t_int = distance_delta/twist_val;
  const double w = twist_val/(2*distance_delta); //2*d so distance_delta = diameter of circle
  const double period = 2*M_PI/w; //testing


  // Set the twist value based on the dimension
  if (dim == "x") {
    twist.twist.linear.x = twist_val;
  } else if (dim == "y") {
    twist.twist.linear.y = twist_val;
  } else if (dim == "z") {
    twist.twist.linear.z = twist_val;
  } else if (dim == "circle") {
    twist.twist.linear.x = twist_val*cos(w*t_); // max twist will still just be twist_val which is already being checked
    twist.twist.linear.y = twist_val*sin(w*t_);
    if (t_ > period) {
      RCLCPP_WARN(kLogger, "Stopping motion. Period (p): %f s", period);
      twist.twist.linear.x = 0;
      twist.twist.linear.y = 0;
      distance_reached_ = true;
    }
  } else if (dim == "square") {
    if (t_ < t_int){
      twist.twist.linear.x = twist_val;
      twist.twist.linear.y = 0;
    } else if (t_ < 2*t_int) {
      twist.twist.linear.x = 0;
      twist.twist.linear.y = twist_val;
    } else if (t_ < 3*t_int) {
      twist.twist.linear.x = -twist_val;
      twist.twist.linear.y = 0;
    } else if (t_ < 4*t_int) {
      twist.twist.linear.x = 0;
      twist.twist.linear.y = -twist_val;
    } else {
      twist.twist.linear.x = 0;
      twist.twist.linear.y = 0;
      distance_reached_ = true;
    }
  } else if (dim == "s") {
    twist.twist.linear.x = twist_val*cos(period*t_); 
    twist.twist.linear.y = twist_val*abs(sin(period*t_));
    // if (t_ > 2*(1/period)) {
    //   RCLCPP_WARN(kLogger, "Stopping motion. Period (p): %f m", period);
    //   twist.twist.linear.x = 0;
    //   twist.twist.linear.y = 0;
    // }
  } else {
    if (dim == "roll")
      twist.twist.angular.z = angular_twist_val_;
    else if (dim == "pitch")
      twist.twist.angular.x = angular_twist_val_;
    else if (dim == "yaw")
      twist.twist.angular.y = angular_twist_val_;
    else {
      RCLCPP_ERROR(kLogger, "Unexpected dimension '%s'. No action taken.",
                   dim.c_str());
    }
  }
}

void HybridMultidimControl::stopMotion() {
  geometry_msgs::msg::TwistStamped twist; // creates empty twist message to send everything to 0
  twist.header.stamp = getNode()->now();
  twist.header.frame_id = force_controller_params_->twist_pub_frame;

  // Publish the twist message
  twist_pub_->publish(twist);
}


BT::NodeStatus HybridMultidimControl::onStart()
{
  // spin ROS node
  // rclcpp::spin_some(getNode());

  // need to lock weak pointers every time you want to use them
  // rclcpp::Node::SharedPtr node = node_ptr_.lock();


  // Get the parameters from the blackboard
  auto params_result = getParams();
  if (!params_result) {
    RCLCPP_ERROR(kLogger, "%s", params_result.error().c_str());
    return BT::NodeStatus::FAILURE;
  }
  // Check that the passed values are valid
  if (!isValidValue(hybrid_motion_primitive_config_->max_twist) ||
      hybrid_motion_primitive_config_->max_twist > kDefaultMaxTwist) {
    RCLCPP_ERROR(kLogger, "Invalid max_twist value: %f",
                 hybrid_motion_primitive_config_->max_twist);
    return BT::NodeStatus::FAILURE;
  }

  if (!isValidValue(hybrid_motion_primitive_config_->distance_delta)) {
    RCLCPP_ERROR(kLogger, "Invalid distance delta value: %f.",
                 hybrid_motion_primitive_config_->distance_delta);
    return BT::NodeStatus::FAILURE;
  }

  if (!isValidDimension(hybrid_motion_primitive_config_->dimension)) {
    RCLCPP_ERROR(kLogger, "Invalid dimension configuration: '%s'.",
                 hybrid_motion_primitive_config_->dimension.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!isValidValue(hybrid_motion_primitive_config_->twist_val)) {
    RCLCPP_ERROR(kLogger, "Invalid twist value configuration: %f.",
                 hybrid_motion_primitive_config_->twist_val);
    return BT::NodeStatus::FAILURE;
  }

  // Update the twist value based on the distance_delta sign
  if (hybrid_motion_primitive_config_->distance_delta < 0) {
    hybrid_motion_primitive_config_->twist_val *= -1;
    angular_twist_val_ *= -1;
  }

  // Create the Force Controller object
  force_controller_ = std::make_shared<twist_force_controller::ForceController>(
      getNode());

  // Initialize the force controller.
  if (!force_controller_->initialize(*force_controller_params_)) {
    RCLCPP_ERROR(kLogger, "Failed to initialize force controller node.");
    return BT::NodeStatus::FAILURE;
  }

  // Create the wrench subscription
  force_controller_->setWrenchSubscription(  
      getNode()
          ->create_subscription<geometry_msgs::msg::WrenchStamped>(
              force_controller_params_->wrench_sub_topic,
              rclcpp::SensorDataQoS(), /*Match the QoS policy of the publisher*/
              std::bind(&HybridMultidimControl::wrenchCallback, this,
                        std::placeholders::_1))); 

  twist_pub_ = getNode()
            ->create_publisher<geometry_msgs::msg::TwistStamped>(
                       force_controller_params_->pub_topic,
                       rclcpp::QoS(rclcpp::KeepLast(1)));

  // Creating publisher to vary wrench data for circles
  wrench_pub_ = getNode()
        ->create_publisher<geometry_msgs::msg::WrenchStamped>(
          force_controller_params_->target_wrench_sub_topic, 
          rclcpp::SensorDataQoS()); // rclcpp::QoS(rclcpp::KeepLast(10))


  // Load the robot model
  if (!initializeRobotModelLoader()) {
    return BT::NodeStatus::FAILURE;
  }

  moveit::core::RobotModelPtr kinematic_model = robot_model_loader_->getModel();
  if (!kinematic_model) {
    RCLCPP_ERROR(kLogger, "Failed to get kinematic model");
    return BT::NodeStatus::FAILURE;
  }

  robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model);
  robot_state_->setToDefaultValues();
  joint_model_group_ = kinematic_model->getJointModelGroup(
      hybrid_motion_primitive_config_->group_name);

  // Subscribe to the joint states topic
  joint_state_sub_ =
      getNode()
          ->create_subscription<sensor_msgs::msg::JointState>(
              "/joint_states", 10,
              std::bind(&HybridMultidimControl::jointStateCallback, this,
                        std::placeholders::_1));

  // Save start time for time-dependent curve calculations
  path_start_time_ = getNode()->now();
  // RCLCPP_WARN(kLogger, "Path start time: %f", path_start_time_);


  /*Create a timer that calls the publish function every 10 milliseconds.*/
  const int time_step{10};
  timer_ = getNode()->create_wall_timer(
      std::chrono::milliseconds(time_step),
      std::bind(&HybridMultidimControl::checkEndEffectorPose, this));

  return BT::NodeStatus::RUNNING;
}

void HybridMultidimControl::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  if(msg==nullptr) {
    RCLCPP_ERROR(kLogger, "Received an invalid joint state message.");
    return;
  }
  current_joint_state_ = msg;
}

bool HybridMultidimControl::initializeRobotModelLoader() {
  try {
    robot_model_loader_ =
        std::make_shared<robot_model_loader::RobotModelLoader>(
            getNode(), "robot_description");

  } catch (const std::exception &e) {
    RCLCPP_ERROR(kLogger, "Failed to initialize RobotModelLoader: %s",
                 e.what());
    return false;
  }
  return true;
}

void HybridMultidimControl::checkEndEffectorPose() {
  if (!initial_pose_set_) { // Start pose tracking when the initial pose is set
    return;
  }
  // Update the robot state with the latest joint states
  robot_state_->setVariablePositions(current_joint_state_->name,
                                     current_joint_state_->position);
  robot_state_->update();

  // publish target wrench
  geometry_msgs::msg::WrenchStamped target_wrench;

  setWrenchValue(target_wrench);
  wrench_pub_->publish(target_wrench);

  const Eigen::Isometry3d &end_effector_state =
      robot_state_->getGlobalLinkTransform(
          force_controller_params_->twist_pub_frame);

  // Convert Eigen::Isometry3d to geometry_msgs::msg::Pose
  geometry_msgs::msg::Pose current_pose;
  tf2::convert(end_effector_state, current_pose);

  // Calculate time elapsed for time-dependent curved paths
  auto now = getNode()->now();
  t_ = (now - path_start_time_).seconds(); 
  RCLCPP_WARN(kLogger, "Timestep (t_): %f s", t_);
  RCLCPP_INFO_STREAM(kLogger, "Distance reached?: " << distance_reached_);


  double distance_moved;
  if (isLinear()) {
    distance_moved = hybrid_multidim_utils::calculateDistanceMoved(current_pose, initial_pose_);


    // RCLCPP_WARN(kLogger, "Robot model frame: %s",
    //         robot_state_->getRobotModel()->getModelFrame().c_str());
            
    if (distance_moved < fabs(hybrid_motion_primitive_config_->distance_delta)) {
      publishTwist();
    } else {
      distance_reached_ = true;
    }
  } else if (isAngular()) {
    distance_moved =
        toDegrees(calculateAngleTurned(current_pose, initial_pose_));
    if (distance_moved < fabs(hybrid_motion_primitive_config_->distance_delta)) {
      publishTwist();
    } else {
      distance_reached_ = true;
    }
  } else if (isShape()) {
    // calculate euclidean difference in position, don't use it to stop trajectory
    distance_moved = hybrid_multidim_utils::calculateDistanceMoved(current_pose, initial_pose_);
    publishTwist(); // logic for stopping twist is wrapped in setTwistValue
  }
}

bool HybridMultidimControl::updateInitialPose() {
  if (!current_joint_state_) {
    RCLCPP_ERROR(kLogger,
                 "No joint state data available. Cannot update initial pose.");
    return false;
  }

  // std::cout << "updating initial pose" << std::endl;

  // Update the robot state with the latest joint states
  robot_state_->setVariablePositions(current_joint_state_->name,
                                     current_joint_state_->position);
  robot_state_->update();

  // std::cout << "initial robot state updated" << std::endl;

  const Eigen::Isometry3d &end_effector_state =
      robot_state_->getGlobalLinkTransform(
          force_controller_params_->twist_pub_frame);

  // Convert and store as initial pose
  tf2::convert(end_effector_state, initial_pose_);
  // std::cout << "set initial pose" << std::endl;

  return true;
}

bool HybridMultidimControl::isLinear() {
  // Convert the input string to lowercase for case-insensitive comparison
  std::string dimLower = hybrid_motion_primitive_config_->dimension;
  std::transform(dimLower.begin(), dimLower.end(), dimLower.begin(), ::tolower);

  return (dimLower == "x" || dimLower == "y" || dimLower == "z");
}

bool HybridMultidimControl::isShape() {
  // Convert the input string to lowercase for case-insensitive comparison
  std::string dimLower = hybrid_motion_primitive_config_->dimension;
  std::transform(dimLower.begin(), dimLower.end(), dimLower.begin(), ::tolower);

  return (dimLower == "circle" || dimLower == "square" || dimLower == "s");
}

bool HybridMultidimControl::isAngular() {
  // Convert the input string to lowercase for case-insensitive comparison
  std::string dimLower = hybrid_motion_primitive_config_->dimension;
  std::transform(dimLower.begin(), dimLower.end(), dimLower.begin(), ::tolower);

  return (dimLower == "roll" || dimLower == "pitch" ||  dimLower == "yaw");
}

void HybridMultidimControl::publishTwist() {

  // Stop if very close to max force/torque or zero wrench
  if (!force_controller_->isValidSensorWrench()) {
    RCLCPP_WARN(kLogger, "Invalid sensor wrench, close to limits or 0");
    stopMotion();
    return;
  }

  geometry_msgs::msg::TwistStamped twist;
  twist.header.stamp = getNode()->now();
  twist.header.frame_id = force_controller_params_->twist_pub_frame;

  // Clamp the twist value to the max_twist boundaries
  hybrid_motion_primitive_config_->twist_val =
      std::clamp(hybrid_motion_primitive_config_->twist_val,
                 -hybrid_motion_primitive_config_->max_twist,
                 hybrid_motion_primitive_config_->max_twist);

  // Set the twist value based on the dimension
  setTwistValue(twist);

  /*If get_controller_twist is set to true, the force controller will just
  compute the twist for every time step, which is grabbed here. Else this is
  published by the force controller node. This node then adds the force
  controller twist to the twist value calculated by the  motion primitive.
  */
  if (force_controller_params_->get_controller_twist) {
    // Add the twist value to the force controller's twist
    geometry_msgs::msg::TwistStamped force_controller_twist =
        force_controller_->getForceControlTwist();

    twist.twist = hybrid_multidim_utils::addTwists(
      twist.twist, force_controller_twist.twist); // Add the twist values
  }

  // Rotate the linear twist 90 deg about the Z axis for Servo in MoveItPro v5.0
  const int rotation_angle{-90};
  rotateLinearTwist(twist.twist, 'z', rotation_angle);

  // Publish the twist message
  twist_pub_->publish(twist);
}


BT::NodeStatus HybridMultidimControl::onRunning()
{ 
  // rclcpp::spin_some(getNode());
  if (!initial_pose_set_) {
    updateInitialPose() == true ? initial_pose_set_ = true
                                : initial_pose_set_ = false;
    start_time_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }

  if (std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::steady_clock::now() - start_time_)
              .count() < hybrid_motion_primitive_config_->publish_time_length &&
      !distance_reached_) {
    // Timeout has not elapsed yet, continue tracking
    return BT::NodeStatus::RUNNING;
  } else {
    onHalted();
    return BT::NodeStatus::SUCCESS;
  }
}

void HybridMultidimControl::onHalted() 
{
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }

  stopMotion();

  joint_state_sub_.reset();
  twist_pub_.reset();

  force_controller_.reset();
  force_controller_params_.reset();
  hybrid_motion_primitive_config_.reset();

  robot_state_.reset();
  robot_model_loader_.reset();
}


HybridMultidimControl::~HybridMultidimControl()
{
  timer_.reset();
  joint_state_sub_.reset();
  twist_pub_.reset();

  if (force_controller_) {
    force_controller_.reset();
  }
}

}  // namespace hybrid_multidim_control

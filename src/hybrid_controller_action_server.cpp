#include "force_controller/hybrid_controller_action_server.hpp"
#include "force_controller/utils/utils.hpp"
#include <chrono>
#include <fenv.h>



namespace hybrid_multidim_control
{

HybridControllerActionServer::HybridControllerActionServer(
    const rclcpp::Node::SharedPtr& node)
    : node_(node) {

  using namespace std::placeholders;
	
	// hybrid_controller is REAL name of action as seen by ROS
	action_server_ = rclcpp_action::create_server<HybridController>(
		node_,
		"hybrid_controller",
		std::bind(&HybridControllerActionServer::handle_goal, this, _1, _2),
		std::bind(&HybridControllerActionServer::handle_cancel, this, _1),
		std::bind(&HybridControllerActionServer::handle_accepted, this, _1),
		rcl_action_server_get_default_options()
	);

		std::cout << "Starting force controller action server..." << std::endl;
}

rclcpp_action::GoalResponse HybridControllerActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const HybridController::Goal> goal)
{
    // make this more robust -> check if goal is valid before accepting
    
	RCLCPP_INFO(node_->get_logger(), "Received goal with following wrench setpoint:");
	controller_utils::logWrenchMsg(goal->wrench_setpoint, node_);

	(void)uuid;
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse HybridControllerActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleHybridController> goal_handle)
{ 
    RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
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
	feenableexcept(FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW);

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<HybridController::Feedback>();
    auto result = std::make_shared<HybridController::Result>();

    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);   feenableexcept(FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW);
        RCLCPP_INFO(node_->get_logger(), "Goal canceled");
        return;
    }

    // Get the parameters from YAML config files and action client
    bool params_result = getParams(goal);
    if (!params_result) {

      RCLCPP_ERROR(node_->get_logger(), "Params not loaded properly.");

      result->success = 0; 
      goal_handle->abort(result);
      return;
    }

    // Create the force controller object
    force_controller_ = std::make_shared<twist_force_controller::ForceController>(node_);

    // Initialize the force controller
    if (!force_controller_->initialize(force_controller_params_)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to initialize force controller.");
      result->success = 0; 
      goal_handle->abort(result);
      return;
    }

	// Initialize robot model -> needed to check EE pose for stopping conditions
	initializeRobotModel();

    // Set up publishers + subscribers
    wrench_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
              params_.wrench_sub_topic,
              rclcpp::SensorDataQoS(), /*Match the QoS policy of the publisher*/
              [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
                // call updateWrench every time a new message comes through
                force_controller_->updateWrench(msg, wrench_data_received_); 
              });

    twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
              params_.pub_topic, rclcpp::QoS(rclcpp::KeepLast(10)));

	// Subscribe to the joint states topic
	joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
				"/joint_states", 10,
				std::bind(&HybridControllerActionServer::jointStateCallback, this,
							std::placeholders::_1));

	// Wait for the FT tare before entering control loop
	RCLCPP_INFO(node_->get_logger(), "Taring FT sensor, keep the arm still...");
	if (!waitForTare(goal_handle, std::chrono::seconds(10))) {

		RCLCPP_ERROR(node_->get_logger(), "FT tare failed or timed out.");
		result->success = 0;

		if (goal_handle->is_canceling()) {
				goal_handle->canceled(result);
			} else {
				goal_handle->abort(result);
			}

		return;
	}

	// Create a timer that calls the publish function every 10ms, enter control loop
    const int time_step{10};
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(time_step),
        std::bind(&HybridControllerActionServer::checkEndEffectorPose, this));	
		

    // Check if node is still running in loop @ 100Hz, keep checking if goal has been met
	rclcpp::Rate loop_rate(100);
    while (rclcpp::ok()) {

		// Cancel action goal if needed
		if (goal_handle->is_canceling()) {
			stopMotion();
			result->message = "Goal canceled";
			goal_handle->canceled(result);
			RCLCPP_INFO(node_->get_logger(), "Goal canceled");
			return;
		}

		if (!initial_pose_set_) {
			updateInitialPose() == true ? initial_pose_set_ = true
										: initial_pose_set_ = false;
			start_time_ = std::chrono::steady_clock::now();
		}

		// Timeout has not elapsed yet, continue tracking and publish some kind of feedback
		if (std::chrono::duration_cast<std::chrono::seconds>(
				std::chrono::steady_clock::now() - start_time_)
					.count() < params_.publish_time_length && 
			!distance_reached_) {

			// goal_handle->publish_feedback(feedback);

		} else {

			// If motion goal is complete, stop and send action result
			stopMotion();

			result->message = "Motion goal completed!";
			result->success = true;
			goal_handle->succeed(result);
			RCLCPP_INFO(node_->get_logger(), "Goal succeeded!");
			return;

		}
	loop_rate.sleep();
    }
}

// --- Helper function definitions ---


bool HybridControllerActionServer::waitForTare(
	const std::shared_ptr<GoalHandleHybridController> goal_handle,
	std::chrono::seconds timeout) 
{
	const auto deadline = std::chrono::steady_clock::now() + timeout;

	while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
		if (goal_handle->is_canceling()) return false;
		if (force_controller_->tareComputed()) return true;
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

return false;
}

HybridGoalParams HybridControllerActionServer::extractGoalParams(
  const std::shared_ptr<const HybridController::Goal>& goal) {

  HybridGoalParams p;
  p.twist_val = goal->twist_val;
  p.distance_delta = goal->distance_delta;
  p.dimension  = goal->dimension;
  p.wrench_setpoint = goal->wrench_setpoint;
  return p;
}

HybridForceExecParams HybridControllerActionServer::buildExecParams(
      const twist_force_controller::ForceControllerParams& force,
      const HybridMotionParams& hybrid,
      const HybridGoalParams& goal)
{
  HybridForceExecParams p{force, hybrid, goal};
  return p;
} 

bool HybridControllerActionServer::getParams(
	std::shared_ptr<const HybridController::Goal> goal) {

	// YAML config params
	force_controller_params_ = twist_force_controller::getControllerParams();
	hybrid_motion_params_ = getHybridMotionParams();

	// BT action client params
	hybrid_goal_params_ = extractGoalParams(goal);

	// Pass goal wrench params into force controller
	geometry_msgs::msg::WrenchStamped stamped_wrench_set;
	stamped_wrench_set.wrench = goal->wrench_setpoint;
	force_controller_params_.wrench_setpoint = controller_utils::toWrenchVector(stamped_wrench_set);

	// combine ALL params into one struct
	params_ = buildExecParams(
	force_controller_params_,
	hybrid_motion_params_,
	hybrid_goal_params_
	);

	return true;
}

void HybridControllerActionServer::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  if(msg==nullptr) {
    RCLCPP_ERROR(node_->get_logger(), "Received an invalid joint state message.");
    return;
  }
  current_joint_state_ = msg;
}

bool HybridControllerActionServer::initializeRobotModel() {

	// Initialize robot model loader
	try {
		robot_model_loader_ =
			std::make_shared<robot_model_loader::RobotModelLoader>(
				node_, "robot_description");

	} catch (const std::exception &e) {
		RCLCPP_ERROR(node_->get_logger(), "Failed to initialize RobotModelLoader: %s",
					e.what());
		return false;
	}

	// Initialize robot kinematics (model)
	moveit::core::RobotModelPtr kinematic_model = robot_model_loader_->getModel();
	if (!kinematic_model) {
		RCLCPP_ERROR(node_->get_logger(), "Failed to get kinematic model.");
		return false;
	}

	// Initialize robot state
	robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model);
	robot_state_->setToDefaultValues();
	joint_model_group_ = kinematic_model->getJointModelGroup(
		params_.group_name);

	if (!robot_state_) {
		RCLCPP_ERROR(node_->get_logger(), "Failed to get robot state.");
		return false;
	}

	return true;
}


void HybridControllerActionServer::checkEndEffectorPose() {
	// Start pose tracking when the initial pose is set and wrench data has been received 
	if (!initial_pose_set_ || !wrench_data_received_) { 
		return;
	}

	// Update the robot state with the latest joint states
	robot_state_->setVariablePositions(current_joint_state_->name,
										current_joint_state_->position);
	robot_state_->update();


	const Eigen::Isometry3d &end_effector_state =
		robot_state_->getGlobalLinkTransform(
			params_.twist_pub_frame);

	// Convert Eigen::Isometry3d to geometry_msgs::msg::Pose
	geometry_msgs::msg::Pose current_pose;
	tf2::convert(end_effector_state, current_pose);

	// Calculate time elapsed for time-dependent curved paths
	auto now = node_->now();
	t_ = (now - path_start_time_).seconds(); 

	double distance_moved;
	if (isLinear()) {
		distance_moved = hybrid_multidim_utils::calculateDistanceMoved(current_pose, initial_pose_);
				
		if (distance_moved < fabs(params_.distance_delta)) {
		publishTwist();
		} else {
		distance_reached_ = true;
		}
	} else if (isAngular()) {
		distance_moved =
			toDegrees(hybrid_multidim_utils::calculateAngleTurned(current_pose, initial_pose_));
		if (distance_moved < fabs(params_.distance_delta)) {
		publishTwist();
		} else {
		distance_reached_ = true;
		}
	} else if (isShape()) {
		distance_moved = hybrid_multidim_utils::calculateDistanceMoved(current_pose, initial_pose_);
		publishTwist();
	} else {
		RCLCPP_WARN(node_->get_logger(), "Publishing twist without checking distance moved.");
		publishTwist();
	}

}

bool HybridControllerActionServer::updateInitialPose() {
	if (!current_joint_state_) {
		RCLCPP_WARN(node_->get_logger(),
					"No joint state data available. Cannot update initial pose.");
		return false;
	} else if (current_joint_state_) {
		RCLCPP_INFO(node_->get_logger(), "Found initial joint state data.");
	}

	if (!robot_state_) {
		RCLCPP_ERROR(node_->get_logger(), "Initial robot_state_ is null!");
		return false;
	}

	// Update the robot state with the latest joint states
	try{ 
		robot_state_->setVariablePositions(current_joint_state_->name,
											current_joint_state_->position);
		robot_state_->update();
	} catch (const std::exception &e) {
		RCLCPP_ERROR(node_->get_logger(), "Failed to update robot state: %s",
					e.what());
		return false;
	}

	const Eigen::Isometry3d &end_effector_state =
		robot_state_->getGlobalLinkTransform(
			params_.twist_pub_frame);

	// Convert and store as initial pose
	tf2::convert(end_effector_state, initial_pose_);

	path_start_time_ = node_->now();

	return true;
}

bool HybridControllerActionServer::isLinear() {
  // Convert the input string to lowercase for case-insensitive comparison
  std::string dimLower = params_.dimension;
  std::transform(dimLower.begin(), dimLower.end(), dimLower.begin(), ::tolower);

  return (dimLower == "x" || dimLower == "y" || dimLower == "z");
}

bool HybridControllerActionServer::isShape() {
  // Convert the input string to lowercase for case-insensitive comparison
  std::string dimLower = params_.dimension;
  std::transform(dimLower.begin(), dimLower.end(), dimLower.begin(), ::tolower);

  return (dimLower == "circle" || dimLower == "square" || dimLower == "s");
}

bool HybridControllerActionServer::isAngular() {
  // Convert the input string to lowercase for case-insensitive comparison
  std::string dimLower = params_.dimension;
  std::transform(dimLower.begin(), dimLower.end(), dimLower.begin(), ::tolower);

  return (dimLower == "roll" || dimLower == "pitch" ||  dimLower == "yaw");
}

void HybridControllerActionServer::setTwistValue(
    geometry_msgs::msg::TwistStamped &twist) {
  const double twist_val = params_.twist_val;
  const double distance_delta = params_.distance_delta;

  // Convert the input dimension to lowercase for case-insensitive comparison
  std::string dim = params_.dimension;
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
  } else {
    if (dim == "roll")
      twist.twist.angular.z = angular_twist_val_;
    else if (dim == "pitch")
      twist.twist.angular.x = angular_twist_val_;
    else if (dim == "yaw")
      twist.twist.angular.y = angular_twist_val_;
    else {
      RCLCPP_ERROR(node_->get_logger(), "Unexpected dimension '%s'. No action taken.",
                  dim.c_str());
    }
  }
}


void HybridControllerActionServer::publishTwist() {

	// Stop if very close to max force/torque or zero wrench
	if (!force_controller_->isValidSensorWrench()) {
		RCLCPP_WARN(node_->get_logger(), "Invalid sensor wrench, close to limits or 0");
		// stopMotion();
		// return;
	}

	geometry_msgs::msg::TwistStamped twist;
	twist.header.stamp = node_->now();
	twist.header.frame_id = params_.twist_pub_frame;

	// Clamp the twist value to the max_twist boundaries
	params_.twist_val =
		std::clamp(params_.twist_val,
					-params_.max_twist,
					params_.max_twist);

	// Set the twist value based on the dimension
	setTwistValue(twist);

	/*If get_controller_twist is set to true, the force controller will just
	compute the twist for every time step, which is grabbed here. Else this is
	published by the force controller node. This node then adds the force
	controller twist to the twist value calculated by the  motion primitive.
	*/

	// Add the twist value to the force controller's twist
	geometry_msgs::msg::TwistStamped force_controller_twist =
		force_controller_->computeTwist();


	twist.twist = hybrid_multidim_utils::addTwists(
	twist.twist, force_controller_twist.twist); // Add the twist values


	// Rotate the linear twist 90 deg about the Z axis for Servo in MoveItPro v5.0
	// const int rotation_angle{-90};
	// hybrid_multidim_utils::rotateLinearTwist(twist.twist, 'z', rotation_angle);

	// controller_utils::logTwistMsg(twist, node_);


	// Publish the twist message to MoveIt servo topic
	// Creates empty twist message to send everything to 0
	// geometry_msgs::msg::TwistStamped empty_twist; 
	// empty_twist.header.stamp = node_->now();
	// empty_twist.header.frame_id = params_.twist_pub_frame;
	
	std::cout << "twist frame: " << twist.header.frame_id << std::endl;

	twist_pub_->publish(twist);


}

void HybridControllerActionServer::stopMotion() {

	if (timer_) {
	timer_->cancel();
	timer_.reset();
	}

	// Creates empty twist message to send everything to 0
	geometry_msgs::msg::TwistStamped twist; 
	twist.header.stamp = node_->now();
	twist.header.frame_id = params_.twist_pub_frame;

	// Publish the twist message 
	twist_pub_->publish(twist);

	// Reset subs/pubs and pointers
	joint_state_sub_.reset();
	twist_pub_.reset();

	force_controller_.reset();
	// params_.reset();

	robot_state_.reset();
	robot_model_loader_.reset();
}

} // namespace hybrid_multidim_control

#include "force_controller/force_controller.hpp"
#include "force_controller/controller_utils.hpp"

namespace twist_force_controller {

constexpr auto ROS_LOG_THROTTLE_PERIOD =
    std::chrono::milliseconds(1000).count();

DimensionHandler::DimensionHandler(const Dimension& dimension)
    : dim(dimension) {
  is_linear = (dim == Dimension::LinX || dim == Dimension::LinY ||
               dim == Dimension::LinZ);
}

void DimensionHandler::setTwistValue(const double val,
                                     geometry_msgs::msg::Twist& twist) const {
  if (is_linear) {
    switch (dim) {
      case Dimension::LinX:
        twist.linear.x = val;
        break;
      case Dimension::LinY:
        twist.linear.y = val;
        break;
      case Dimension::LinZ:
        twist.linear.z = val;
        break;
    }
  }
}

double DimensionHandler::getWrenchValue(
    const geometry_msgs::msg::WrenchStamped& cur_wrench) const {
  if (is_linear) {
    switch (dim) {
      case Dimension::LinX:
        return cur_wrench.wrench.force.x;
      case Dimension::LinY:
        return cur_wrench.wrench.force.y;
      case Dimension::LinZ:
        return cur_wrench.wrench.force.z;
    }
  }
  return 0.0;
}

// Constructor
ForceController::ForceController(const rclcpp::Node::SharedPtr& node)
    : Node("force_controller"),
      previous_error_(0.0),
      integral_(0.0),
      prev_time_(0.0),
      delta_time_(0.0),
      node_(node) {}

// Destructor
ForceController::~ForceController() {
  setTwistToZero();
  if(wrench_sub_){wrench_sub_.reset();}
  if(target_wrench_sub_){target_wrench_sub_.reset();}
  if(target_force_sub_) {target_force_sub_.reset(); }
  if(twist_pub_){twist_pub_.reset();}
  if(wrench_pub_){wrench_pub_.reset();}
  if(timer_) {timer_->cancel(); timer_.reset(); } 
  if(node_){node_.reset(); }
}

Dimension ForceController::convertToDimension(const std::string& dim) {
  if (dim == "x" || dim == "X")
    return Dimension::LinX;
  else if (dim == "y" || dim == "Y")
    return Dimension::LinY;
  else
    return Dimension::LinZ;
}

bool ForceController::setTargetForce(const double target_force) {
  // Check if target_force is non-zero & within a valid range
  if (isValidTargetForce(target_force)) {
    target_force_ = target_force; /*Update the target force*/
    RCLCPP_INFO(node_->get_logger(), "Target force set to %f", target_force_);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Invalid target force value: %f",
                 target_force);
    return false;
  }
  return true;
}

bool ForceController::setTargetWrench(
    const std::vector<double>& target_wrench) {
  // Check if target_wrench is valid
  if (isValidTargetWrench(target_wrench)) {
    target_wrench_ = target_wrench; /*Update the target wrench*/
    controller_utils::logWrenchVector(target_wrench_, node_->get_name(),
                                      "Target wrench set to");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Invalid target wrench value");
    return false;
  }
  return true;
}

bool ForceController::setControllerParams(
    const std::vector<double>& param_val, const std::string& param_name,
    std::vector<double>& controller_param) {
  const double minimum_threshold = 1e-3;
  for (auto& param : param_val) {
    if (param <= minimum_threshold) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Invalid %s value; must be > 0. Aborting initialization.",
                   param_name.c_str());

      return false;
    }
  }

  if (control_wrench_) {
    if (param_val.size() != NUM_DIMS) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Invalid %s size. Aborting initialization.",
                   param_name.c_str());
      return false;
    }
    controller_param.resize(NUM_DIMS, 0);
    controller_param = param_val;
  } else {
    controller_param = param_val;
  }
  return true;
}

bool ForceController::setDamping(const std::vector<double>& damping) {
  return setControllerParams(damping, "damping", damping_);
}

bool ForceController::setStiffness(const std::vector<double>& stiffness) {
  return setControllerParams(stiffness, "stiffness", stiffness_);
}

bool ForceController::setMaxTwist(const double max_twist) {
  const double minimum_threshold{1e-3};
  if (max_twist <= minimum_threshold) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "Invalid max twist value; must be > 0. Aborting initialization.");
    return false;
  }

  const double maximum_threshold{0.5};
  if (max_twist > maximum_threshold) {
    RCLCPP_WARN(node_->get_logger(),
                "Max twist value exceeds the maximum allowed value. Setting to "
                "the maximum allowed value.");
    max_twist_ = maximum_threshold;
    return true;
  }

  max_twist_ = max_twist;
  return true;
}

bool ForceController::initialize(const ForceControllerParams& params) {
  RCLCPP_INFO(node_->get_logger(), "Initializing force controller node . . .");

  force_controller_params_ = params;

  if (!force_controller_params_.has_ft_sensor) {
    RCLCPP_WARN(node_->get_logger(), "No FT sensor present. Aborting.");
    return false;
  }

  // Get the required parameters
  kp_ = force_controller_params_.kp;
  ki_ = force_controller_params_.ki;
  kd_ = force_controller_params_.kd;
  max_force_ = force_controller_params_.max_force;
  max_torque_ = force_controller_params_.max_torque;
  max_twist_ = force_controller_params_.max_twist;
  dimension_ = convertToDimension(force_controller_params_.dimension_str);
  ff_gain_ = force_controller_params_.ff_gain;
  anti_windup_error_threshold_ =
      force_controller_params_.anti_windup_error_threshold;
  control_wrench_ = force_controller_params_.control_wrench;
  ft_sensor_frame_ = force_controller_params_.ft_sensor_frame;
  gravity_frame_ = force_controller_params_.gravity_frame;
  twist_pub_frame_ = force_controller_params_.twist_pub_frame;
  compensate_for_ee_weight_ = force_controller_params_.compensate_for_ee_weight;
  use_PID_ = force_controller_params_.use_PID;
  filter_params_.deadband_threshold =
      force_controller_params_.filter_params.deadband_threshold;
  filter_params_.apply_ema_filter =
      force_controller_params_.filter_params.apply_ema_filter;
  filter_params_.alpha = force_controller_params_.filter_params.alpha;
  filter_params_.apply_bw_filter =
      force_controller_params_.filter_params.apply_bw_filter;
  filter_params_.cut_off_frequency =
      force_controller_params_.filter_params.cut_off_frequency;
  filter_params_.dt = force_controller_params_.filter_params.dt;
  filter_params_.apply_kf_filter =
      force_controller_params_.filter_params.apply_kf_filter;
  filter_params_.process_noise =
      force_controller_params_.filter_params.process_noise;
  filter_params_.measurement_noise =
      force_controller_params_.filter_params.measurement_noise;
  filter_params_.filter_order =
      force_controller_params_.filter_params.filter_order;

  // Handle 1D/6D control parameters
  if (control_wrench_) {
    if (!setTargetWrench(force_controller_params_.wrench_setpoint)) {
      return false;
    }

    if (!setDamping(force_controller_params_.damping_6D)) {
      return false;
    }

    if (!setStiffness(force_controller_params_.stiffness_6D)) {
      return false;
    }

    displacement_.assign(NUM_DIMS, 0.0);  // Init displacement vector to 6D

    target_wrench_sub_ = /*Setup the callback*/
        node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
            force_controller_params_.target_wrench_sub_topic,
            rclcpp::QoS(rclcpp::KeepLast(10)),
            std::bind(&ForceController::targetWrenchCB, this,
                      std::placeholders::_1));
  } else {
    if (!setTargetForce(force_controller_params_.force_setpoint)) {
      return false;
    }

    if (!setDamping(force_controller_params_.damping_1D)) {
      return false;
    }

    if (!setStiffness(force_controller_params_.stiffness_1D)) {
      return false;
    }

    displacement_.assign(1, 0.0);  // Init displacement vector to 1D

    target_force_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
        force_controller_params_.target_force_sub_topic,
        rclcpp::QoS(rclcpp::KeepLast(10)),
        std::bind(&ForceController::targetForceCB, this,
                  std::placeholders::_1));
  }

  // Set the maximum twist
  if (!setMaxTwist(force_controller_params_.max_twist)) {
    return false;
  }

  if (!force_controller_params_.add_restoring_force_term) {
    for (auto& stiff : stiffness_) {
      stiff = 0.0;
    }
  }

  if (compensate_for_ee_weight_) {
    // Get the EE weight in the gravitational frame
    ee_weight_.vector.z = -1 * force_controller_params_.ee_weight;

    // Get and validate the EE CoM
    ee_com_ = force_controller_params_.ee_com;
    const int com_dim{3};
    if (ee_com_.size() != com_dim) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Invalid EE CoM size. Aborting initialization.");
      return false;
    }

    // Setup the transform listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      force_controller_params_.pub_topic, rclcpp::QoS(rclcpp::KeepLast(10)));

  // For demonstration purposes in RQT
  wrench_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "wrench_pub_topic", rclcpp::QoS(rclcpp::KeepLast(10)));

  return true;
}

void ForceController::updateFTCB(
    const geometry_msgs::msg::WrenchStamped::SharedPtr wrench) {
  // Verify that the wrench data is valid
  if (controller_utils::isWrenchNan(wrench)) {
    RCLCPP_WARN(node_->get_logger(),
                "Invalid wrench (NaN) data received. Ignoring input.");
    return;
  }

  cur_wrench_ = *wrench;

  if (!tare_computed_) {
    // Collect 1000 samples for taring
    tare_data_.push_back(cur_wrench_.wrench);
    if (tare_data_.size() > tare_samples_) {
      tare_data_.pop_front();
    }
    if (tare_data_.size() == tare_samples_) {
      avg_wrench_ = ft_sensor_utils::computeAverageFtsTare(tare_data_);
      tare_computed_ = true;
      start_time_ = std::chrono::steady_clock::now();
    }
  }

  // Tare the FTS readings
  if (tare_computed_) {
    // Apply the tare correction to the force torque sensor readings
    ft_sensor_utils::applyTareCorrection(avg_wrench_, cur_wrench_.wrench);

    // Track the time for a reset
    const int reset_time = 5;
     if (std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::steady_clock::now() - start_time_).count() > reset_time){
            tare_computed_ = false;
          }
  }

  // Publish the tared wrench data for visualization
  wrench_pub_->publish(cur_wrench_);

  if (compensate_for_ee_weight_) {
    // Get the updated ee_weight in the ft_sensor_frame
    if (!ft_sensor_utils::transformVector(ee_weight_, *tf_buffer_,
                                          ft_sensor_frame_, gravity_frame_))
      return;

    // Get the wrench due to the ee_weight in the ft_sensor_frame
    geometry_msgs::msg::WrenchStamped ee_weight_wrench;
    ee_weight_wrench.header = cur_wrench_.header;
    ft_sensor_utils::getEEWeightWrench(ee_weight_wrench, ee_weight_, ee_com_);

    // Bias the sensor data by the EE weight
    ft_sensor_utils::biasWrench(ee_weight_wrench, cur_wrench_);
  }
}

void ForceController::targetForceCB(
    const std_msgs::msg::Float64::SharedPtr target_force) {
  if (!setTargetForce(target_force->data)) {
    RCLCPP_WARN(node_->get_logger(),
                "Invalid target force value received. Ignoring input.");
    return;
  }
}

void ForceController::targetWrenchCB(
    const geometry_msgs::msg::WrenchStamped::SharedPtr target_wrench) {
  if (!setTargetWrench(controller_utils::toWrenchVector(target_wrench))) {
    RCLCPP_WARN(node_->get_logger(),
                "Invalid target wrench value received. Ignoring input.");
    return;
  }
}

bool ForceController::isValidTargetForce(const double& target_force) const {
  // Check if target_force is a valid number
  if (std::isnan(target_force)) {
    RCLCPP_WARN(node_->get_logger(),
                "Invalid target force value received. Ignoring input.");
    return false;
  }

  // Check if target_force is within the valid range
  if (!controller_utils::isInRange(target_force, -max_force_, max_force_)) {
    RCLCPP_WARN(node_->get_logger(),
                "Target force value out of force range. Ignoring input.");
    return false;
  }

  return true;
}

bool ForceController::isValidTargetWrench(
    const std::vector<double>& target_wrench) const {
  if (target_wrench.size() != NUM_DIMS) {
    RCLCPP_WARN(node_->get_logger(),
                "Invalid target wrench size received. Make sure to set the "
                "correct force and torque fields. Ignoring input.");
    return false;
  }

  if (controller_utils::isWrenchNan(target_wrench)) {
    RCLCPP_WARN(node_->get_logger(),
                "Invalid target wrench (NaN) value received. Ignoring input.");
    return false;
  }

  // Check if target_wrench is within the valid range
  if (!controller_utils::isWrenchInRange(target_wrench, max_force_,
                                         max_torque_)) {
    RCLCPP_WARN(node_->get_logger(),
                "Target wrench value out of range. Ignoring input.");
    return false;
  }

  return true;
}

bool ForceController::isValidSensorWrench() {

  const double max_wrench_percent{0.9};
        
  if (
    std::abs(cur_wrench_.wrench.force.x) > max_wrench_percent * max_force_ ||
    std::abs(cur_wrench_.wrench.force.y) > max_wrench_percent * max_force_ ||
    std::abs(cur_wrench_.wrench.force.z) > max_wrench_percent * max_force_ ||
    std::abs(cur_wrench_.wrench.torque.x) > max_wrench_percent * max_torque_ ||
    std::abs(cur_wrench_.wrench.torque.y) > max_wrench_percent * max_torque_ ||
    std::abs(cur_wrench_.wrench.torque.z) > max_wrench_percent * max_torque_
  ) {
      RCLCPP_ERROR_STREAM_THROTTLE(
        node_->get_logger(), *node_->get_clock(), ROS_LOG_THROTTLE_PERIOD,
        "Current force or torque close to wrench limits. Stopping motion!");

      return false;
  }

  // Check if the wrench data is zero
  double wrench_mag = wrenchMagnitude(cur_wrench_);
  const double wrench_threshold = 1e-6;
  if (fabs(wrench_mag) < wrench_threshold) {
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(),
                                ROS_LOG_THROTTLE_PERIOD,
                                "Zero wrench data received. Ignoring input.");
    return false;
  }
  return true;
}

void ForceController::publishTwist() {
  // Stop if very close to max force/torque or zero wrench
  if (!isValidSensorWrench()) {
    std::cout << "Invalid sensor wrench from force controller!" << std::endl;
    setTwistToZero();
    return;
  }
  
  if(!tare_computed_) return;

  filter_utils::FilterUtils filter;  // Create a filter object

  if (!control_wrench_) {  // Simple, 1D case
    //  Construct the dimension object in each iteration to ensure that the
    //  dimension is updated
    DimensionHandler dim_handler(dimension_);
    double cur_wrench = dim_handler.getWrenchValue(cur_wrench_);

    // Apply filters
    filter.filterWrench(cur_wrench, filter_params_);

    // Compute the desired twist
    double twist_desired = computeTwist({cur_wrench}).at(0);

    // Clamp the desired twist to the twist max
    twist_desired = fabs(twist_desired) > max_twist_
                        ? clamp(twist_desired, -max_twist_, max_twist_)
                        : twist_desired;

    // Set the twist value
    dim_handler.setTwistValue(twist_desired, twist_.twist);
  } else {  // 6D case
    // Apply filters
    filter.filterWrench(cur_wrench_, filter_params_);
    filtered_wrench_ = cur_wrench_;

    // Compute the desired twist
    std::vector<double> twist_desired =
        computeTwist(controller_utils::toWrenchVector(cur_wrench_));

    // Clamp the desired twist to the twist max
    for (size_t i = 0; i < NUM_DIMS; i++) {
      twist_desired.at(i) =
          (fabs(twist_desired.at(i)) > max_twist_)
              ? clamp(twist_desired.at(i), -max_twist_, max_twist_)
              : twist_desired.at(i);
    }

    // Set the twist value
    controller_utils::toTwistMsg(twist_, twist_desired);
  }

  // Publish the desired twist
  twist_.header.stamp = node_->get_clock()->now();
  twist_.header.frame_id = twist_pub_frame_;

  if (!force_controller_params_.get_controller_twist) {
    twist_pub_->publish(twist_);
  }
}

void ForceController::setTwistToZero() {
  twist_.twist = geometry_msgs::msg::Twist();
  twist_.header.stamp = node_->get_clock()->now();
  if(twist_pub_)twist_pub_->publish(twist_);
}

double ForceController::computeControlSignal(const double measured_force) {
  /*PID control: The proportional term adjusts the control action based on the
     current error, the integral term integrates past errors to eliminate
     steady-state errors, and the derivative term anticipates future errors
     based on the rate of change of the error.*/

  //  Get the time delta
  double cur_time = node_->get_clock()->now().seconds();
  delta_time_ = cur_time - prev_time_;

  //  Handle the first iter when prev_time_ is zero
  if (delta_time_ > TIME_STEP) {
    delta_time_ = TIME_STEP;
  }

  // Compute the error
  double error = target_force_ - measured_force;
  //In Moveit Pro implementation, it seems that this is needed to ensure 
  //that the function is triggered for all 6D values
  // RCLCPP_WARN(node_->get_logger(), "target force  %f", target_force_);  
  
  /*Compute the integral term
  Apply anti-windup limit to prevent the integral term
  from accumulating excessively and causing overshoot or instability. This
  usually happens if the system is saturated/unable to achieve the desired
  output.*/
  if (fabs(error) < anti_windup_error_threshold_) {  // Anti-windup
    integral_ += error * delta_time_;
  }

  // Compute the derivative term
  double derivative = (error - previous_error_) / delta_time_;

  // Compute the control signal, u(t), using the PID control law
  double control_signal = kp_ * error + ki_ * integral_ + kd_ * derivative;

  previous_error_ = error;  // Update the previous error
  prev_time_ = cur_time;    // Update the previous time

  // Compute the feedforward term
  double feedforward_term{0.0};
  feedforward_term = ff_gain_ * target_force_;

  return control_signal + feedforward_term;
}

std::vector<double> ForceController::computeForceError(
    const std::vector<double>& measured_wrench) {
  std::vector<double> force_error(measured_wrench.size(), 0.0);

  for (size_t i = 0; i < measured_wrench.size(); i++) {
    if (measured_wrench.size() == target_wrench_.size()) {
      force_error.at(i) = target_wrench_.at(i) - measured_wrench.at(i);
    } else {
      force_error.at(i) = target_force_ - measured_wrench.at(i);
    }
  }
  return force_error;
}

std::vector<double> ForceController::computeTwist(
    const std::vector<double>& measured_wrench) {
  /*
  Assumption: The robot is in full contact with the environment.

  Main equation:  v = (F_error - kx) / b
  where, F_target is the desired force, k is the stiffness, x is the
  displacement, b is the damping, and v is the velocity. This ensures that
  both the restoring force, kx, and the damping force, bv, act in directions
  that oppose displacement and velocity, respectively, leading to a stable
  equilibrium and controlled motion.
  */
  std::vector<double> twist(measured_wrench.size(), 0.0);

  // Initialize previous error signal vector
  if (prev_error_signal_.empty()) {
    prev_error_signal_.resize(measured_wrench.size(), 0.0);
  }

  // Compute the twist
  for (size_t i = 0; i < measured_wrench.size(); i++) {
    double error_signal{0.0};
    if (use_PID_) { 
      if(control_wrench_){
        target_force_ = target_wrench_.at(i);
        error_signal = computeControlSignal(measured_wrench.at(i));
        // std::cout << "error signal: " << error_signal << std::endl;

      }
      else{
        DimensionHandler dim_handler(dimension_);
        error_signal = computeControlSignal(dim_handler.getWrenchValue(cur_wrench_));
      }
    }
    else {
      error_signal = kp_ * computeForceError(measured_wrench).at(i);
    }

    twist.at(i) = -1 *
                  (error_signal - (stiffness_.at(i) * displacement_.at(i))) /
                  damping_.at(i);
      // std::cout << "raw impedance twist " << twist.at(i) << std::endl;

    

    // Cap the twist value to avoid excessively large updates
    if (fabs(twist.at(i)) > max_twist_) {
      twist.at(i) = std::copysign(max_twist_, twist.at(i));
    }
    // Update the displacement
    displacement_.at(i) += twist.at(i) * delta_time_;

    // Reset condition for displacement: small twist, small error signal, or
    // error signal sign change
    if (fabs(twist.at(i)) < 1e-4 || fabs(error_signal) < 1e-4 ||
        (prev_error_signal_.at(i) * error_signal < 0)) {
      displacement_.at(i) = 0.0;
    }

    // Update previous error signal
    prev_error_signal_.at(i) = error_signal;
  }
  return twist;
}

}  // namespace twist_force_controller

/************************************************************************************
 *      Title       : force_controller.hpp
 *      Project     : force_controller
 *      Created     : 11/08/2023
 *      Author      : Emmanuel Akita
 *      Email       : efakita@utexas.edu
 *      Description : This is a twist-based force controller. It commands an
 *                    end-effector twist to achieve a target force based on
 *                    the force feedback in the set dimension. This controls
 *                    the force exerted at the end effector of the robot arm.
 *                    It leverages a simple PID controller setup for control
 *                    and a wrench filter to smooth the wrench data.
 *      Copyright   : Copyright© The University of Texas at Austin, 2014-2023.
 *                    All rights reserved.
 *
 *          All files within this directory are subject to the following, unless
 *          an alternative license is explicitly included within the text of
 *           each file.
 *
 *          This software and documentation constitute an unpublished work
 *          and contain valuable trade secrets and proprietary information
 *          belonging to the University. None of the foregoing material may be
 *          copied or duplicated or disclosed without the express, written
 *          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
 *          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
 *          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 *          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
 *          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
 *          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
 *          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
 *          University be liable for incidental, special, indirect, direct or
 *          consequential damages or loss of profits, interruption of business,
 *          or related expenses which may arise from use of software or
 *          documentation, including but not limited to those resulting from
 *          defects in software and/or documentation, or loss or inaccuracy of
 *          data of any kind.
 *
 **********************************************************************************/

#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <deque>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include "force_controller/utils/wrench_filters.hpp"
#include "force_controller/utils/param_handler.hpp"
#include "force_controller/utils/controller_utils.hpp"


namespace twist_force_controller {


const size_t NUM_DIMS{6};      // 3 translational, 3 rotational dimensions
const double TIME_STEP{0.01};  // 10 milliseconds


class ForceController {
 public:
  ForceController(const rclcpp::Node::SharedPtr& node);

  /**
   * @brief Sets the target force for the controller
   * @details It ensures the target force is valid before setting it
   * @param target_force The target force value
   * @return Returns true if the target force is valid and set, false otherwise
   */
  bool setTargetForce(const double target_force);

  /**
   * @brief Sets the target wrench for the controller
   * @details It ensures the target wrench is valid before setting it
   * @param target_wrench The target wrench value
   * @return Returns true if the target wrench is valid and set, false otherwise
   */
  bool setTargetWrench(const std::vector<double>& target_wrench);

  /**
   * @brief Sets the damping for the controller
   * @details It ensures the damping is valid before setting it
   * @param damping The damping value
   * @return Returns true if the damping is valid and set, false otherwise
   */
  bool setDamping(const std::vector<double>& damping);

  /**
   * @brief Sets the stiffness for the controller
   * @details It ensures the stiffness is valid before setting it
   * @param stiffness The stiffness value
   * @return Returns true if the stiffness is valid and set, false otherwise
   */
  bool setStiffness(const std::vector<double>& stiffness);

  /**
   * @brief Sets the model parameters for the controller
   * @details It ensures the model parameters are valid before setting them
   * @param param_value The model parameter value
   * @param param_name The model parameter name
   * @param controller_param The controller class member variable to be updated
   * with the new parameter value
   * @return Returns true if the model parameter is valid and set, false
   * otherwise
   */
  bool setControllerParams(const std::vector<double>& param_val,
                           const std::string& param_name,
                           std::vector<double>& controller_param);

  /**
   * @brief Sets the maximum twist for the controller
   * @details It ensures the maximum twist is valid before setting it
   * @param max_twist The maximum twist value to be set for the controller
   * @return Returns true if the maximum twist is valid and set, false otherwise
   */
  bool setMaxTwist(const double max_twist);

  /** @brief Sets & publishes 0 twist. */
  void setTwistToZero();

  /** @brief Sets the timer for the controller*/
  void setTimer(const rclcpp::TimerBase::SharedPtr& timer) { timer_ = timer; }

  /**
   * @brief Callback function for the wrench data subscriber
   * @param wrench The wrench data message
   * @post The wrench data is updated
   */
  void updateWrench(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);

  /** @brief Triggers the private targetForceCB function */
  void triggerStopMotion() { setTwistToZero(); }

  /** @brief Grabs the parameters, sets the parameters and sets up the callbacks
   */
  bool initialize(const ForceControllerParams& params);

  /**
   * @brief Publish twist_ values for force control
   * @details It computes the desired twist based on PID control and the target
   * force setpoint in every cycle iteration
    @note For safety, it checks the wrench magnitude does not exceed a 90% of
      given max, and clamps the twist value to the max twist value
   * @post The twist message is published
   */
  geometry_msgs::msg::TwistStamped computeTwist();

  /**
   * @brief Checks to ensure the measured sensor wrench is valid
   * @details  It ensures the wrench doesn't get dangerously high or
   * isn't zero. For high wrenches, it ensures the wrench magnitude does not
   * exceed 90% of given max.
   * @note. This is used for incoming force-torque data
   * @return True if wrenches are valid, false otherwise
   */
  bool isValidSensorWrench();

  ~ForceController();

 private:

  /**
   * @brief Checks to ensure the target force is valid
   * @details It ensures the target force is not zero and is within the max and
   * min force range
   * @return True if target force is valid, false otherwise
   */
  bool isValidTargetForce(const double& target_force) const;

  /**
   * @brief Checks to ensure the target wrench is valid
   * @details It ensures that the correct number of wrench elements are
   * provided, that none of the target wrench values is zero and that all values
   * are within the max and min wrench range.
   * @return True if target wrench is valid, false otherwise
   */
  bool isValidTargetWrench(const std::vector<double>& target_wrench) const;

  /**
   * @brief Computes the magnitude of the wrench
   * @param wrench The wrench data message
   * @return The magnitude of the wrench
   */
  inline double wrenchMagnitude(
      const geometry_msgs::msg::WrenchStamped& wrench) {
    return sqrt(pow(wrench.wrench.force.x, 2) + pow(wrench.wrench.force.y, 2) +
                pow(wrench.wrench.force.z, 2) + pow(wrench.wrench.torque.x, 2) +
                pow(wrench.wrench.torque.y, 2) +
                pow(wrench.wrench.torque.z, 2));
  }

  /**
   * @brief Computes the control signal based on the measured and target force.
   * @details It uses a PID controller for the computation based on the force
   * error. The PID controller is tuned using the parameters kp_, kd_ and ki_.
   * @param measured_force The measured force in the set dimension
   * @return The computed twist value
   */
  double computePIDControlSignal(const double measured_force);

  /**
   * @brief Computes the force error based on the measured force and target
   * force
   * @param measured_force The measured force in the set dimension
   * @return The computed force error
   */
  std::vector<double> computeForceError(
      const std::vector<double>& measured_wrench);

  /**
   * @brief Computes the twist based on the measured and target wrench.
   * @details It uses a PID controller to compute the twist value based on the
   * force error. The PID controller is tuned using the parameters kp_, kd_
   * and ki_.
   * @note This is implemented for both 1D and 6D force control via using
   * vectors
   * @param measured_wrench The measured wrench or force in the set dimension
   * @return The computed twist as a vector.
   */
  std::vector<double> computeAdmittanceLaw(const std::vector<double>& measured_wrench);

  /**
   * @brief Clamps the value between the min and max values
   * @note: This template function can be used by different data types
   * @param value The value to be clamped
   * @param min The minimum value
   * @param max The maximum value
   * @return The clamped value
   */
  template <typename T>
  T clamp(const T& value, const T& min, const T& max) {
    return std::min(std::max(value, min), max);
  }

  geometry_msgs::msg::WrenchStamped cur_wrench_, tare_values_, filtered_wrench_;
  geometry_msgs::msg::Wrench avg_wrench_;
  geometry_msgs::msg::TwistStamped twist_;

  rclcpp::TimerBase::SharedPtr timer_;

  bool control_wrench_, compensate_for_ee_weight_, use_PID_;

  double ff_gain_, max_twist_, target_force_, max_force_, max_torque_,
      previous_error_, integral_, kp_, kd_, ki_, prev_time_, delta_time_,
      anti_windup_error_threshold_;

  filter_utils::FilterWrenchParams filter_params_;

  std::vector<double> target_wrench_, damping_, stiffness_, ee_com_,
      displacement_;

  controller_utils::Dimension dimension_;

  std::string ft_sensor_frame_, gravity_frame_, twist_pub_frame_;

  geometry_msgs::msg::Vector3Stamped ee_weight_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  ForceControllerParams force_controller_params_;
  rclcpp::Node::SharedPtr node_;

  bool tare_computed_{false};
  size_t tare_samples_{1000};
  std::deque<geometry_msgs::msg::Wrench> tare_data_;
  std::vector<double> prev_error_signal_;
  std::chrono::time_point<std::chrono::steady_clock> start_time_;

};
}  // namespace twist_force_controller
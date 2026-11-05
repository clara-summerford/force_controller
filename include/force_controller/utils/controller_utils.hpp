#pragma once
#include <tf2_ros/buffer.h>
#include <deque>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace controller_utils {

enum Dimension {
  LinX,
  LinY,
  LinZ,
};

struct DimensionHandler {
  DimensionHandler(const Dimension& dimension);

  // Returns the correct dimension enum
  bool isLinear() const { return is_linear; };

  bool isAngular() const { return !is_linear; };

  // Sets the correct field in the twist message
  void setTwistValue(const double val, geometry_msgs::msg::Twist& twist) const;

  // Returns the correct measured wrench value based on the dimension
  double getWrenchValue(
      const geometry_msgs::msg::WrenchStamped& cur_wrench) const;

 private:
  Dimension dim;
  bool is_linear;
};

/**
 * @brief Converts string to dimension enums
 * @param dim String corresponding to the required dimension
 * @return The enum corresponding to the string passed.
 */
Dimension convertToDimension(const std::string& dim);

/**
 * @brief Checks whether the passed value is a valid number value
 * @param wrench The value to be checked
 * @return True if the value is a valid number, false otherwise
 */
bool isWrenchNan(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);

/**
 * @brief Checks whether the passed value is a valid number value
 * @note This is an overloaded function, which takes a wrench vector as input
 * @param wrench The value to be checked
 * @return True if the value is a valid number, false otherwise
 */
bool isWrenchNan(const std::vector<double>& wrench);

/**
 * @brief Converts a wrench message to a wrench vector (Fx, Fy, Fz, Tx, Ty, Tz)
 * @param wrench The wrench message
 * @return The wrench vector
 */
std::vector<double> toWrenchVector(
    const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);

/**
 * @brief Converts a wrench message to a wrench vector (Fx, Fy, Fz, Tx, Ty, Tz)
 * @note This is an overloaded function, which takes a wrench message as input
 * @param wrench The wrench message
 * @return The wrench vector
 */
std::vector<double> toWrenchVector(
    const geometry_msgs::msg::WrenchStamped& wrench);

/**
 * @brief Checks whether the passed value is within the specified range
 * @param value The value to be checked
 * @param min The absolute minimum value of the range
 * @param max The absolute maximum value of the range
 * @return True if the value is within the range, false otherwise
 */
bool isInRange(const double value, const double min, const double max);

/**
 * @brief Checks whether the passed wrench is within the specified range
 * @param wrench The wrench to be checked
 * @param max_force The maximum force value of the range
 * @param max_torque The maximum torque value of the range
 * @return True if the wrench is within the range, false otherwise
 */
bool isWrenchInRange(const std::vector<double>& wrench, const double max_force,
                     const double max_torque);

/**
 * @brief Converts a wrench vector to a twist message
 * @param twist The twist message
 * @param value The wrench vector
 * @post twist is updated with the wrench vector
 */
void toTwistMsg(geometry_msgs::msg::TwistStamped& twist,
                const std::vector<double>& value);

/**
 * @brief  * This function logs the values of a wrench vector using the
 * specified logger name and message
 * @param wrench The wrench vector to log
 * @param logger_name The logger name
 * @param msg The message to include in the log statement
 * @post The wrench vector is logged using the specified logger name and message
 */
void logWrenchVector(const std::vector<double>& wrench,
                     rclcpp::Node::SharedPtr node,
                     const std::string& msg = "Wrench Vector");


/**
 * @brief Logs all forces and torques from a wrench message.
 * 
 * @param wrench 
 * @param node 
 */
void logWrenchMsg(const geometry_msgs::msg::Wrench& wrench, 
              rclcpp::Node::SharedPtr node,
                const std::string& msg = "Wrench");


/**
 * @brief Logs twist values from a twist stampted message.
 * 
 * @param twist 
 * @param node 
 */
void logTwistMsg(const geometry_msgs::msg::TwistStamped& twist, 
                rclcpp::Node::SharedPtr node,
                const std::string& msg = "Twist");


/**
 * @brief Computes the magnitude of the wrench
 * @param wrench The wrench data message
 * @return The magnitude of the wrench
 */
double wrenchMagnitude(const geometry_msgs::msg::WrenchStamped& wrench);   

}  // namespace controller_utils

namespace ft_sensor_utils {
// Handles data operations for the ft sensor data

/**
 * @brief Transforms the input vector from the source frame to the target frame
 * @param vector The input vector
 * @param tf_buffer The tf buffer
 * @param to_frame The target frame
 * @param from_frame The source frame
 * @return True if the transform was successful, false otherwise
 */
bool transformVector(geometry_msgs::msg::Vector3Stamped& vector,
                     const tf2_ros::Buffer& tf_buffer,
                     const std::string& to_frame,
                     const std::string& from_frame);

/**
 * @brief Calculates the weight wrench of the end effector
 * @details This is the end effector weight, and the associated torque it causes
 * @param ee_weight_wrench The end effector weight wrench object
 * @param ee_weight The weight of the end effector
 * @param ee_com The center of mass of the end effector
 * @post ee_weight_wrench is updated with the weight wrench of the end effector
 */
void getEEWeightWrench(geometry_msgs::msg::WrenchStamped& ee_weight_wrench,
                       const geometry_msgs::msg::Vector3Stamped& ee_weight,
                       const std::vector<double>& ee_com);

/**
 * @brief Applies a bias to the input wrench data
 * @param ee_weight_wrench The bias
 * @param cur_wrench The current wrench data
 * @post cur_wrench is updated with the bias
 */
void biasWrench(
    const geometry_msgs::msg::WrenchStamped& ee_weight_wrench, /*bias*/
    geometry_msgs::msg::WrenchStamped& cur_wrench);

/**
 * @brief Tares the sensor by computing the average of several readings to
 * determine the tare value.
 * @param tare_data The data to be used for taring
 * @return The average tare value
 */
geometry_msgs::msg::Wrench computeAverageFtsTare(
    const std::deque<geometry_msgs::msg::Wrench>& tare_data);

/**
 * @brief Apply the tare correction to the force torque sensor readings.
 * @param avg_wrench The average wrench value
 * @param ft_values The force torque sensor readings
 * @post ft_values is updated with the tare correction
 */
void applyTareCorrection(const geometry_msgs::msg::Wrench& avg_tare_wrench,
                         geometry_msgs::msg::Wrench& ft_values);

}  // namespace ft_sensor_utils

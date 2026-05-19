#include "force_controller/controller_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // tf2::doTransform
namespace controller_utils {
bool isWrenchNan(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench) {
  return std::isnan(wrench->wrench.force.x) ||
         std::isnan(wrench->wrench.force.y) ||
         std::isnan(wrench->wrench.force.z) ||
         std::isnan(wrench->wrench.torque.x) ||
         std::isnan(wrench->wrench.torque.y) ||
         std::isnan(wrench->wrench.torque.z);
}

bool isWrenchNan(const std::vector<double>& wrench) {
  return std::isnan(wrench.at(0)) || std::isnan(wrench.at(1)) ||
         std::isnan(wrench.at(2)) || std::isnan(wrench.at(3)) ||
         std::isnan(wrench.at(4)) || std::isnan(wrench.at(5));
}
std::vector<double> toWrenchVector(
    const geometry_msgs::msg::WrenchStamped::SharedPtr wrench) {
  return {wrench->wrench.force.x,  wrench->wrench.force.y,
          wrench->wrench.force.z,  wrench->wrench.torque.x,
          wrench->wrench.torque.y, wrench->wrench.torque.z};
}

std::vector<double> toWrenchVector(
    const geometry_msgs::msg::WrenchStamped& wrench) {
  return {wrench.wrench.force.x,  wrench.wrench.force.y,
          wrench.wrench.force.z,  wrench.wrench.torque.x,
          wrench.wrench.torque.y, wrench.wrench.torque.z};
}
bool isInRange(const double value, const double min, const double max) {
  return (value >= min && value <= max);
}

bool isWrenchInRange(const std::vector<double>& wrench, const double max_force,
                     const double max_torque) {
  double min_force = -max_force;
  double min_torque = -max_torque;

  if (!isInRange(wrench.at(0), min_force, max_force) ||
      !isInRange(wrench.at(1), min_force, max_force) ||
      !isInRange(wrench.at(2), min_force, max_force) ||
      !isInRange(wrench.at(3), min_torque, max_torque) ||
      !isInRange(wrench.at(4), min_torque, max_torque) ||
      !isInRange(wrench.at(5), min_torque, max_torque)) {
    return false;
  }
  return true;
}

void toTwistMsg(geometry_msgs::msg::TwistStamped& twist,
                const std::vector<double>& value) {
  twist.twist.linear.x = value.at(0);
  twist.twist.linear.y = value.at(1);
  twist.twist.linear.z = value.at(2);
  twist.twist.angular.x = value.at(3);
  twist.twist.angular.y = value.at(4);
  twist.twist.angular.z = value.at(5);
}

void toWrenchMsg(geometry_msgs::msg::WrenchStamped& wrench,
                const std::vector<double>& value) {
  wrench.wrench.force.x = value.at(0);
  wrench.wrench.force.y = value.at(1);
  wrench.wrench.force.z = value.at(2);
  wrench.wrench.torque.x = value.at(3);
  wrench.wrench.torque.y = value.at(4);
  wrench.wrench.torque.z = value.at(5);
}

void logWrenchVector(const std::vector<double>& wrench,
                     const std::string& logger_name, const std::string& msg) {
  RCLCPP_INFO(rclcpp::get_logger(logger_name), "%s: [%f, %f, %f, %f, %f, %f]",
              msg.c_str(), wrench.at(0), wrench.at(1), wrench.at(2),
              wrench.at(3), wrench.at(4), wrench.at(5));
}
}  // namespace controller_utils

namespace ft_sensor_utils {

bool transformVector(geometry_msgs::msg::Vector3Stamped& vector,
                     const tf2_ros::Buffer& tf_buffer,
                     const std::string& to_frame,
                     const std::string& from_frame) {
  //  Get the transformation object
  geometry_msgs::msg::TransformStamped transform_stamped; /*gravity_to_ft_tf*/

  try {
    transform_stamped =
        tf_buffer.lookupTransform(to_frame, from_frame, tf2::TimePointZero, tf2::durationFromSec(3.0));
    RCLCPP_INFO(rclcpp::get_logger("ft_sensor_utils"), "Transform found!");
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(rclcpp::get_logger("ft_sensor_utils"), "%s", ex.what());
    return false;
  }

  // Transform the ee_weight from the gravity frame to the ft_sensor_frame
  tf2::doTransform(vector, vector, transform_stamped);
  return true;
}

void getEEWeightWrench(geometry_msgs::msg::WrenchStamped& ee_weight_wrench,
                       const geometry_msgs::msg::Vector3Stamped& ee_weight,
                       const std::vector<double>& ee_com) {
  // Get the force component of the ee_weight
  ee_weight_wrench.wrench.force.x = ee_weight.vector.x;
  ee_weight_wrench.wrench.force.y = ee_weight.vector.y;
  ee_weight_wrench.wrench.force.z = ee_weight.vector.z;

  // Calculate the torques due to the ee_weight using the CoM information
  // Torque = (ee_r_com) cross (ee_weight)
  ee_weight_wrench.wrench.torque.x =
      (ee_com.at(1) * ee_weight.vector.z) - (ee_com.at(2) * ee_weight.vector.y);
  ee_weight_wrench.wrench.torque.y =
      (ee_com.at(2) * ee_weight.vector.x) - (ee_com.at(0) * ee_weight.vector.z);
  ee_weight_wrench.wrench.torque.z =
      (ee_com.at(0) * ee_weight.vector.y) - (ee_com.at(1) * ee_weight.vector.x);
}

void biasWrench(const geometry_msgs::msg::WrenchStamped& ee_weight_wrench,
                geometry_msgs::msg::WrenchStamped& cur_wrench) {
  cur_wrench.wrench.force.x -= ee_weight_wrench.wrench.force.x;
  cur_wrench.wrench.force.y -= ee_weight_wrench.wrench.force.y;
  cur_wrench.wrench.force.z -= ee_weight_wrench.wrench.force.z;
  cur_wrench.wrench.torque.x -= ee_weight_wrench.wrench.torque.x;
  cur_wrench.wrench.torque.y -= ee_weight_wrench.wrench.torque.y;
  cur_wrench.wrench.torque.z -= ee_weight_wrench.wrench.torque.z;
}

geometry_msgs::msg::Wrench computeAverageFtsTare(
    const std::deque<geometry_msgs::msg::Wrench>& tare_data) {
  geometry_msgs::msg::Wrench tare_values;

  // Calculate average values for taring
  for (auto& data : tare_data) {
    tare_values.force.x += data.force.x;
    tare_values.force.y += data.force.y;
    tare_values.force.z += data.force.z;
    tare_values.torque.x += data.torque.x;
    tare_values.torque.y += data.torque.y;
    tare_values.torque.z += data.torque.z;
  }
  int count = tare_data.size();
  tare_values.force.x /= count;
  tare_values.force.y /= count;
  tare_values.force.z /= count;
  tare_values.torque.x /= count;
  tare_values.torque.y /= count;
  tare_values.torque.z /= count;

  return tare_values;
}

void applyTareCorrection(const geometry_msgs::msg::Wrench& avg_wrench,
                         geometry_msgs::msg::Wrench& ft_values) {
  // Apply the tare correction to the force torque sensor readings
  ft_values.force.x -= avg_wrench.force.x;
  ft_values.force.y -= avg_wrench.force.y;
  ft_values.force.z -= avg_wrench.force.z;
  ft_values.torque.x -= avg_wrench.torque.x;
  ft_values.torque.y -= avg_wrench.torque.y;
  ft_values.torque.z -= avg_wrench.torque.z;
}
}  // namespace ft_sensor_utils
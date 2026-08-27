
#include "glovebox_bt/utils/utils.hpp"
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace hybrid_multidim_utils {
geometry_msgs::msg::Twist addTwists(const geometry_msgs::msg::Twist &twist1,
                                    const geometry_msgs::msg::Twist &twist2) {
  geometry_msgs::msg::Twist result;

  // Add the linear components
  result.linear.x = twist1.linear.x + twist2.linear.x;
  result.linear.y = twist1.linear.y + twist2.linear.y;
  result.linear.z = twist1.linear.z + twist2.linear.z;

  // Add the angular components
  result.angular.x = twist1.angular.x + twist2.angular.x;
  result.angular.y = twist1.angular.y + twist2.angular.y;
  result.angular.z = twist1.angular.z + twist2.angular.z;

  return result;
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

void rotateLinearTwist(geometry_msgs::msg::Twist &linear_twist, char axis,
                       double theta_degrees) {
  // Convert the angle to radians
  double theta_radians = theta_degrees * M_PI / 180.0;

  // Create the rotation matrix
  Eigen::Matrix3d rotation_matrix;
  switch (axis) {
  case 'x':
    rotation_matrix =
        Eigen::AngleAxisd(theta_radians, Eigen::Vector3d::UnitX());
    break;
  case 'y':
    rotation_matrix =
        Eigen::AngleAxisd(theta_radians, Eigen::Vector3d::UnitY());
    break;
  case 'z':
    rotation_matrix =
        Eigen::AngleAxisd(theta_radians, Eigen::Vector3d::UnitZ());
    break;
  default:
    RCLCPP_ERROR(rclcpp::get_logger("CartesianMotionPrimitives"),
                 "Invalid axis. Use 'x', 'y', or 'z'.");
    return;
  }

  // Rotate the vector
  Eigen::Vector3d linear_twist_vec(linear_twist.linear.x, linear_twist.linear.y,
                                   linear_twist.linear.z);
  linear_twist_vec = rotation_matrix * linear_twist_vec;

  // Update the linear_twist
  linear_twist.linear.x = linear_twist_vec[0];
  linear_twist.linear.y = linear_twist_vec[1];
  linear_twist.linear.z = linear_twist_vec[2];
}

double calculateDistanceMoved(const geometry_msgs::msg::Pose &current_pose,
                              const geometry_msgs::msg::Pose &initial_pose) {
  // Compute the Euclidean distance between the initial and current poses
  double dx = current_pose.position.x - initial_pose.position.x;
  double dy = current_pose.position.y - initial_pose.position.y;
  double dz = current_pose.position.z - initial_pose.position.z;

  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double calculateAngleTurned(const geometry_msgs::msg::Pose &current_pose,
                            const geometry_msgs::msg::Pose &initial_pose) {
  tf2::Quaternion initial_q, current_q, delta_q;
  double angle;

  // Convert to tf2::Quaternion
  tf2::convert(initial_pose.orientation, initial_q);
  tf2::convert(current_pose.orientation, current_q);

  // Normalize quaternions to ensure they represent valid rotations
  initial_q.normalize();
  current_q.normalize();

  // Calculate the relative rotation delta between the initial and current
  // orientations
  delta_q = initial_q.inverse() * current_q;
  delta_q.normalize(); // Normalize the delta to ensure mathematical correctness

  // Calculate the angle of rotation represented by the quaternion delta
  angle = delta_q.getAngle();

  return angle;
}
} // namespace hybrid_multidim_utils